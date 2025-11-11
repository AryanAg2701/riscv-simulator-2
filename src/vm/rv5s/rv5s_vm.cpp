/**
 * @file rv5s_vm.cpp
 * @brief RV5S VM implementation - 5-Stage Pipeline (Mode 2)
 */

#include "vm/rv5s/rv5s_vm.h"

#include "utils.h"
#include "globals.h"
#include "common/instructions.h"
#include "config.h"
#include "vm/vm_base.h"

#include <cctype>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <tuple>
#include <stack>  
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

RV5SVM::RV5SVM() : VmBase(), branch_predictor_(BranchPredictor::ALWAYS_NOT_TAKEN) {
  // Initialize hazard detection and forwarding from config
  hazard_detection_enabled_ = vm_config::config.getHazardDetection();
  forwarding_enabled_ = vm_config::config.getForwarding();
  branch_prediction_enabled_ = vm_config::config.getBranchPrediction();
  
  // Safety check: Forwarding requires hazard detection (Mode 4)
  if (forwarding_enabled_ && !hazard_detection_enabled_) {
    // Forwarding without hazard detection can cause incorrect results
    // Disable forwarding if hazard detection is not enabled
    forwarding_enabled_ = false;
  }
  
  // Initialize branch prediction policy
  std::string policy_str = vm_config::config.getBranchPredictionPolicy();
  if (policy_str == "always_taken") {
    branch_predictor_.SetPolicy(BranchPredictor::ALWAYS_TAKEN);
  } else {
    branch_predictor_.SetPolicy(BranchPredictor::ALWAYS_NOT_TAKEN);
  }
  
  DumpRegisters(globals::registers_dump_file_path, registers_);
  // DumpState is not called here because program_ is not loaded yet
  // It will be called after LoadProgram() in VmBase::LoadProgram()
}

RV5SVM::~RV5SVM() = default;

void RV5SVM::RequestStop() {
  stop_requested_ = true;
  VmBase::RequestStop();  // Also call base class
}

bool RV5SVM::IsStopRequested() const {
  return stop_requested_ || VmBase::stop_requested_;
}

void RV5SVM::ClearStop() {
  stop_requested_ = false;
  VmBase::stop_requested_ = false;
}

bool RV5SVM::HasInstructionsInPipeline() const {
  return if_id.valid || id_ex.valid || ex_mem.valid || mem_wb.valid || 
         program_counter_ < program_size_;
}

void RV5SVM::InsertBubble() {
  id_ex.Clear();
  id_ex.instruction = 0x00000013;
  id_ex.valid = true;
}

void RV5SVM::FlushPipeline() {
  if_id.Clear();
  id_ex.Clear();
  pipeline_flush_ = false;
}

void RV5SVM::Tick() {
  // Execute stages in reverse order (WB → MEM → EX → ID → IF)
  // This ensures data flows correctly through pipeline registers
  
  WriteBack();      // Stage 5: Write back from MEM/WB
  MemoryAccess();   // Stage 4: Memory access from EX/MEM
  Execute();        // Stage 3: Execute from ID/EX
  Decode();         // Stage 2: Decode from IF/ID
  Fetch();          // Stage 1: Fetch new instruction
  
  cycle_s_++;
}

void RV5SVM::Fetch() {
  // Check if we should stall
  if (pipeline_stall_) {
    // Don't fetch new instruction, keep current PC
    // Insert NOP into IF/ID to prevent invalid instruction
    if_id.instruction = 0x00000013;  // NOP
    if_id.pc = program_counter_;  // Keep current PC
    if_id.valid = true;
    return;
  }
  
  // Check if pipeline should be flushed (branch taken)
  if (pipeline_flush_) {
    FlushPipeline();
    // After flush, fetch from new PC (set by Execute stage)
    if (program_counter_ < program_size_) {
      if_id.instruction = memory_controller_.ReadWord(program_counter_);
      if_id.pc = program_counter_;
      if_id.valid = true;
      UpdateProgramCounter(4);
    } else {
      if_id.Clear();
    }
    return;
  }
  
  // Fetch instruction from memory
  if (program_counter_ < program_size_) {
    if_id.instruction = memory_controller_.ReadWord(program_counter_);
    if_id.pc = program_counter_;
    if_id.valid = true;
    UpdateProgramCounter(4);
  } else {
    if_id.Clear();
  }
}

void RV5SVM::Decode() {
  // Read from IF/ID register
  if (!if_id.valid) {
    id_ex.Clear();
    return;
  }
  
  uint32_t instruction = if_id.instruction;
  uint64_t pc = if_id.pc;
  
  // Set control signals
  control_unit_.SetControlSignals(instruction);
  
  // Extract fields
  uint8_t rs1 = (instruction >> 15) & 0b11111;
  uint8_t rs2 = (instruction >> 20) & 0b11111;
  uint8_t rd = (instruction >> 7) & 0b11111;
  int32_t imm = ImmGenerator(instruction);
  
  // Mode 3/4: Hazard Detection (with optional forwarding)
  if (hazard_detection_enabled_) {
    bool should_stall = false;
    
    // Mode 4: Check if forwarding can resolve the hazard
    if (forwarding_enabled_) {
      // Create temporary ID/EX register with current instruction's registers
      // to check forwarding availability
      ID_EX_Register temp_id_ex;
      temp_id_ex.rs1 = rs1;
      temp_id_ex.rs2 = rs2;
      temp_id_ex.rd = rd;
      temp_id_ex.read_data1 = registers_.ReadGpr(rs1);
      temp_id_ex.read_data2 = registers_.ReadGpr(rs2);
      
      // Get forwarding signals to see if we can forward
      auto [forwardA, forwardB] = forwarding_unit_.GetForwardingSignals(temp_id_ex, ex_mem, mem_wb);
      
      // Load-use hazards always need a stall (can't forward immediately)
      // Check mem_wb because when Decode() runs, MemoryAccess() has already
      // moved the load from ex_mem to mem_wb
      if (hazard_unit_.DetectLoadUseHazard(if_id, mem_wb)) {
        should_stall = true;
        stats_.data_hazards++;  // Load-use hazard
      } 
      // Check if there's a data hazard that forwarding can't resolve
      else if (hazard_unit_.DetectDataHazard(if_id, id_ex, ex_mem, mem_wb)) {
        // If forwarding is available, no stall needed
        if (forwardA == ForwardingUnit::FORWARD_NONE && forwardB == ForwardingUnit::FORWARD_NONE) {
          // No forwarding available, must stall
          should_stall = true;
          stats_.data_hazards++;
        } else {
          // Forwarding available - no stall needed
          should_stall = false;
        }
      } else {
        should_stall = false;
      }
    } else {
      // Mode 3: No forwarding, use standard hazard detection
      should_stall = hazard_unit_.ShouldStall(if_id, id_ex, ex_mem, mem_wb);
      if (should_stall) {
        if (hazard_unit_.DetectLoadUseHazard(if_id, mem_wb)) {
          stats_.data_hazards++;  // Load-use hazard
        } else {
          stats_.data_hazards++;  // General RAW hazard
        }
      }
    }
    
    if (should_stall) {
      // Hazard detected - insert bubble and stall pipeline
      pipeline_stall_ = true;
      InsertBubble();  // Insert NOP 
      stats_.stalls++;
      return;
    } else {
      // No hazard
      pipeline_stall_ = false;
    }
  }
  
  // Read register file
  uint64_t read_data1 = registers_.ReadGpr(rs1);
  uint64_t read_data2 = registers_.ReadGpr(rs2);
  
  // Mode 5: Static Branch Prediction
  id_ex.branch_predicted = false;
  id_ex.branch_predicted_taken = false;
  id_ex.branch_predicted_target = 0;
  
  if (branch_prediction_enabled_ && BranchPredictor::IsBranchOrJump(instruction)) {
    // Predict branch outcome
    bool predicted_taken = branch_predictor_.Predict(instruction, pc);
    id_ex.branch_predicted = true;
    id_ex.branch_predicted_taken = predicted_taken;
    
    if (predicted_taken) {
      // Calculate predicted target
      uint64_t predicted_target = branch_predictor_.CalculateBranchTarget(instruction, pc);
      
      // Extract opcode for JALR check
      uint8_t opcode = instruction & 0x7F;
      
      // Special handling for JALR (needs rs1 value)
      if (BranchPredictor::IsUnconditionalJump(instruction) && 
          (opcode == get_instr_encoding(Instruction::kjalr).opcode)) {
        // For JALR, we need rs1 value to calculate target
        // Use forwarded value if available, otherwise read from register
        uint64_t rs1_value = read_data1;
        if (forwarding_enabled_) {
          ID_EX_Register temp_id_ex;
          temp_id_ex.rs1 = rs1;
          temp_id_ex.rs2 = rs2;
          temp_id_ex.read_data1 = read_data1;
          temp_id_ex.read_data2 = read_data2;
          auto [forwardA, forwardB] = forwarding_unit_.GetForwardingSignals(temp_id_ex, ex_mem, mem_wb);
          if (forwardA != ForwardingUnit::FORWARD_NONE) {
            rs1_value = forwarding_unit_.GetForwardedValueA(forwardA, temp_id_ex, ex_mem, mem_wb);
          }
        }
        // Calculate JALR target: (rs1 + imm) & ~1
        int32_t jalr_imm = ImmGenerator(instruction);
        predicted_target = (rs1_value + static_cast<uint64_t>(static_cast<int64_t>(jalr_imm))) & ~1ULL;
      }
      
      id_ex.branch_predicted_target = predicted_target;
      
      // Update PC to predicted target (will be used in next Fetch)
      program_counter_ = predicted_target;
      stats_.branch_predictions++;
    } else {
      // Predict not taken - continue sequential execution
      id_ex.branch_predicted_target = pc + 4;  // Sequential
      stats_.branch_predictions++;
    }
  }
  
  // Update ID/EX register
  id_ex.reg_write = control_unit_.GetRegWrite();
  id_ex.mem_to_reg = control_unit_.GetMemToReg();
  id_ex.branch = control_unit_.GetBranch();
  id_ex.mem_read = control_unit_.GetMemRead();
  id_ex.mem_write = control_unit_.GetMemWrite();
  id_ex.alu_src = control_unit_.GetAluSrc();
  id_ex.alu_op = control_unit_.GetAluOp();
  
  id_ex.pc = pc;
  id_ex.read_data1 = read_data1;
  id_ex.read_data2 = read_data2;
  id_ex.imm = imm;
  id_ex.rs1 = rs1;
  id_ex.rs2 = rs2;
  id_ex.rd = rd;
  id_ex.instruction = instruction;
  id_ex.valid = true;
}

void RV5SVM::Execute() {
  // Read from ID/EX register
  if (!id_ex.valid) {
    ex_mem.Clear();
    return;
  }
  
  uint32_t instruction = id_ex.instruction;
  uint8_t opcode = instruction & 0b1111111;
  uint8_t funct3 = (instruction >> 12) & 0b111;
  
  // Handle syscalls
  if (opcode == get_instr_encoding(Instruction::kecall).opcode && 
      funct3 == get_instr_encoding(Instruction::kecall).funct3) {
    HandleSyscall();
    ex_mem.Clear();
    return;
  }
  
  // // Handle floating point instructions
  // if (instruction_set::isFInstruction(instruction)) {
  //   ExecuteFloat();
  //   return;
  // } else if (instruction_set::isDInstruction(instruction)) {
  //   ExecuteDouble();
  //   return;
  // } else if (opcode == 0b1110011) {
  //   ExecuteCsr();
  //   return;
  // }
  
  // Get ALU inputs with forwarding (Mode 4)
  uint64_t reg1_value = id_ex.read_data1;
  uint64_t reg2_value = id_ex.read_data2;
  
  // Apply forwarding if enabled
  if (forwarding_enabled_) {
    auto [forwardA, forwardB] = forwarding_unit_.GetForwardingSignals(id_ex, ex_mem, mem_wb);
    
    // Get forwarded value for rs1 (operand A)
    reg1_value = forwarding_unit_.GetForwardedValueA(forwardA, id_ex, ex_mem, mem_wb);
    
    // Get forwarded value for rs2 (operand B)
    reg2_value = forwarding_unit_.GetForwardedValueB(forwardB, id_ex, ex_mem, mem_wb);
    
    // Track forwarding events for statistics
    if (forwardA != ForwardingUnit::FORWARD_NONE || forwardB != ForwardingUnit::FORWARD_NONE) {
      stats_.forwarding_events++;
    }
  }
  
  // Use immediate if alu_src is true (after forwarding)
  if (id_ex.alu_src) {
    reg2_value = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));
  }
  
  // Perform ALU operation
  alu::AluOp aluOperation = control_unit_.GetAluSignal(instruction, id_ex.alu_op);
  bool overflow = false;
  std::tie(execution_result_, overflow) = alu_.execute(aluOperation, reg1_value, reg2_value);
  
  // Handle branches and jumps
  bool branch_taken = false;
  uint64_t branch_target = 0;
  bool mispredicted = false;
  
  if (id_ex.branch) {
    if (opcode == get_instr_encoding(Instruction::kjalr).opcode || 
        opcode == get_instr_encoding(Instruction::kjal).opcode) {
      return_address_ = id_ex.pc + 4;
      if (opcode == get_instr_encoding(Instruction::kjalr).opcode) {
        branch_target = execution_result_;
        branch_taken = true;
      } else if (opcode == get_instr_encoding(Instruction::kjal).opcode) {
        branch_target = id_ex.pc + id_ex.imm;
        branch_taken = true;
      }
      
      // Mode 5: Check for misprediction on unconditional jumps
      if (branch_prediction_enabled_ && id_ex.branch_predicted) {
        // Unconditional jumps are always taken, so if we predicted not taken, it's a misprediction
        if (!id_ex.branch_predicted_taken || id_ex.branch_predicted_target != branch_target) {
          mispredicted = true;
          stats_.branch_mispredictions++;
          pipeline_flush_ = true;
          program_counter_ = branch_target;
        } else {
          // Correct prediction - no flush needed, PC already updated
          pipeline_flush_ = false;
        }
      } else {
        // Mode 2-4: Original behavior (always flush on jump)
        pipeline_flush_ = true;
        program_counter_ = branch_target;
      }
    } else if (opcode == get_instr_encoding(Instruction::kbeq).opcode ||
               opcode == get_instr_encoding(Instruction::kbne).opcode ||
               opcode == get_instr_encoding(Instruction::kblt).opcode ||
               opcode == get_instr_encoding(Instruction::kbge).opcode ||
               opcode == get_instr_encoding(Instruction::kbltu).opcode ||
               opcode == get_instr_encoding(Instruction::kbgeu).opcode) {
      // Conditional branches
      switch (funct3) {
        case 0b000: branch_taken = (execution_result_ == 0); break; // BEQ
        case 0b001: branch_taken = (execution_result_ != 0); break; // BNE
        case 0b100: branch_taken = (execution_result_ == 1); break; // BLT
        case 0b101: branch_taken = (execution_result_ == 0); break; // BGE
        case 0b110: branch_taken = (execution_result_ == 1); break; // BLTU
        case 0b111: branch_taken = (execution_result_ == 0); break; // BGEU
      }
      
      if (branch_taken) {
        branch_target = id_ex.pc + id_ex.imm;
      } else {
        branch_target = id_ex.pc + 4;  // Sequential
      }
      
      // Mode 5: Check for misprediction
      if (branch_prediction_enabled_ && id_ex.branch_predicted) {
        bool predicted_correctly = (id_ex.branch_predicted_taken == branch_taken) &&
                                   (id_ex.branch_predicted_target == branch_target);
        
        if (!predicted_correctly) {
          // Misprediction!
          mispredicted = true;
          stats_.branch_mispredictions++;
          pipeline_flush_ = true;
          program_counter_ = branch_target;  // Fetch from correct address
        } else {
          // Correct prediction - no flush needed
          pipeline_flush_ = false;
          // PC already updated by prediction in Decode stage
        }
      } else {
        // Mode 2-4: Original behavior (flush on taken branch)
        if (branch_taken) {
          pipeline_flush_ = true;
          program_counter_ = branch_target;
        } else {
          pipeline_flush_ = false;
        }
      }
    }
  }
  
  // Handle AUIPC and LUI
  if (opcode == get_instr_encoding(Instruction::kauipc).opcode) {
    execution_result_ = static_cast<int64_t>(id_ex.pc) + (id_ex.imm << 12);
  } else if (opcode == get_instr_encoding(Instruction::klui).opcode) {
    execution_result_ = id_ex.imm << 12;
  }
  
  // Update EX/MEM register
  ex_mem.reg_write = id_ex.reg_write;
  ex_mem.mem_to_reg = id_ex.mem_to_reg;
  ex_mem.mem_read = id_ex.mem_read;
  ex_mem.mem_write = id_ex.mem_write;
  ex_mem.branch_target = branch_target;
  ex_mem.branch_taken = branch_taken;
  ex_mem.branch_mispredicted = mispredicted;
  ex_mem.alu_result = execution_result_;
  
  // For stores: use forwarded value if forwarding is enabled
  if (forwarding_enabled_ && id_ex.mem_write) {
    auto [forwardA, forwardB] = forwarding_unit_.GetForwardingSignals(id_ex, ex_mem, mem_wb);
    ex_mem.write_data = forwarding_unit_.GetForwardedStoreData(forwardB, id_ex, ex_mem, mem_wb);
  } else {
    ex_mem.write_data = id_ex.read_data2;  // For stores
  }
  
  ex_mem.pc = id_ex.pc;  // Pass PC through for return address calculation
  ex_mem.rd = id_ex.rd;
  ex_mem.instruction = instruction;
  ex_mem.valid = true;
}

void RV5SVM::MemoryAccess() {
  // Read from EX/MEM register
  if (!ex_mem.valid) {
    mem_wb.Clear();
    return;
  }
  
  uint32_t instruction = ex_mem.instruction;
  uint8_t opcode = instruction & 0b1111111;
  uint8_t funct3 = (instruction >> 12) & 0b111;
  uint8_t rs2 = (instruction >> 20) & 0b11111;
  
  if (opcode == 0b1110011 && funct3 == 0b000) {
    mem_wb.Clear();
    return;
  }
  
  // Handle floating point memory operations
  if (instruction_set::isFInstruction(instruction)) {
    WriteMemoryFloat();
    return;
  } else if (instruction_set::isDInstruction(instruction)) {
    WriteMemoryDouble();
    return;
  }
  
  // Perform memory read
  if (ex_mem.mem_read) {
    switch (funct3) {
      case 0b000: memory_result_ = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem.alu_result)); break;
      case 0b001: memory_result_ = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem.alu_result)); break;
      case 0b010: memory_result_ = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem.alu_result)); break;
      case 0b011: memory_result_ = memory_controller_.ReadDoubleWord(ex_mem.alu_result); break;
      case 0b100: memory_result_ = static_cast<uint8_t>(memory_controller_.ReadByte(ex_mem.alu_result)); break;
      case 0b101: memory_result_ = static_cast<uint16_t>(memory_controller_.ReadHalfWord(ex_mem.alu_result)); break;
      case 0b110: memory_result_ = static_cast<uint32_t>(memory_controller_.ReadWord(ex_mem.alu_result)); break;
    }
  }
  
  // Perform memory write
  if (ex_mem.mem_write) {
    uint64_t write_val = ex_mem.write_data;
    switch (funct3) {
      case 0b000: memory_controller_.WriteByte(ex_mem.alu_result, write_val & 0xFF); break;
      case 0b001: memory_controller_.WriteHalfWord(ex_mem.alu_result, write_val & 0xFFFF); break;
      case 0b010: memory_controller_.WriteWord(ex_mem.alu_result, write_val & 0xFFFFFFFF); break;
      case 0b011: memory_controller_.WriteDoubleWord(ex_mem.alu_result, write_val); break;
    }
  }
  
  // Update MEM/WB register
  mem_wb.reg_write = ex_mem.reg_write;
  mem_wb.mem_to_reg = ex_mem.mem_to_reg;
  mem_wb.alu_result = ex_mem.alu_result;
  mem_wb.mem_read_data = memory_result_;
  mem_wb.pc = ex_mem.pc;  // Pass PC through for return address calculation
  mem_wb.rd = ex_mem.rd;
  mem_wb.instruction = instruction;
  mem_wb.valid = true;
}

void RV5SVM::WriteBack() {
  // Read from MEM/WB register
  if (!mem_wb.valid) {
    return;
  }
  
  uint32_t instruction = mem_wb.instruction;
  uint8_t opcode = instruction & 0b1111111;
  uint8_t funct3 = (instruction >> 12) & 0b111;
  uint8_t rd = mem_wb.rd;
  
  // Count all instructions (including stores, floating point, CSR, etc.)
  // Skip NOPs (0x00000013) and invalid instructions
  bool should_count = (instruction != 0 && instruction != 0x00000013);
  
  if (opcode == get_instr_encoding(Instruction::kecall).opcode && 
      funct3 == get_instr_encoding(Instruction::kecall).funct3) {
    if (should_count) {
      instructions_retired_++;
      stats_.instructions_retired++;
    }
    mem_wb.Clear();
    return;
  }
  
  // Handle floating point writeback
  if (instruction_set::isFInstruction(instruction)) {
    WriteBackFloat();
    if (should_count) {
      instructions_retired_++;
      stats_.instructions_retired++;
    }
    mem_wb.Clear();
    return;
  } else if (instruction_set::isDInstruction(instruction)) {
    WriteBackDouble();
    if (should_count) {
      instructions_retired_++;
      stats_.instructions_retired++;
    }
    mem_wb.Clear();
    return;
  } else if (opcode == 0b1110011) {
    WriteBackCsr();
    if (should_count) {
      instructions_retired_++;
      stats_.instructions_retired++;
    }
    mem_wb.Clear();
    return;
  }
  
  // Write back to register file
  if (mem_wb.reg_write && rd != 0) {  // x0 is always 0
    uint64_t write_value = 0;
    
    if (mem_wb.mem_to_reg) {
      write_value = mem_wb.mem_read_data;
    } else {
      write_value = mem_wb.alu_result;
    }
    
    // Handle special cases (JAL/JALR return address = PC + 4)
    if (opcode == get_instr_encoding(Instruction::kjal).opcode ||
        opcode == get_instr_encoding(Instruction::kjalr).opcode) {
      write_value = mem_wb.pc + 4;  // Return address stored in pipeline register
    }
    // LUI and AUIPC results are already in alu_result from Execute stage
    
    registers_.WriteGpr(rd, write_value);
  }
  
  // Count all instructions (including stores and other non-register-writing instructions)
  // Skip NOPs (0x00000013) and invalid instructions
  if (instruction != 0 && instruction != 0x00000013) {
    instructions_retired_++;
    stats_.instructions_retired++;
  }
  
  // Clear pipeline register after processing
  mem_wb.Clear();
}

// Floating point and CSR methods (simplified - can be expanded)
void RV5SVM::ExecuteFloat() {
  // Simplified - delegate to base functionality
  // Full implementation would be similar to RVSSVM::ExecuteFloat()
  ex_mem.Clear();
}

void RV5SVM::ExecuteDouble() {
  ex_mem.Clear();
}

void RV5SVM::ExecuteCsr() {
  ex_mem.Clear();
}

void RV5SVM::WriteMemoryFloat() {
  mem_wb.Clear();
}

void RV5SVM::WriteMemoryDouble() {
  mem_wb.Clear();
}

void RV5SVM::WriteBackFloat() {
  // Placeholder
}

void RV5SVM::WriteBackDouble() {
  // Placeholder
}

void RV5SVM::WriteBackCsr() {
  // Placeholder
}

void RV5SVM::HandleSyscall() {
  // Simplified syscall handling
  uint64_t syscall_number = registers_.ReadGpr(17);
  switch (syscall_number) {
    case SYSCALL_EXIT:
      stop_requested_ = true;
      break;
    default:
      break;
  }
  ex_mem.Clear();
}

void RV5SVM::Run() {
  ClearStop();
  stats_ = PipelineStats();  // Reset statistics

  while (!stop_requested_ && !VmBase::stop_requested_ && HasInstructionsInPipeline()) {
    if (stats_.instructions_retired > vm_config::config.getInstructionExecutionLimit())
      break;

    Tick();
    stats_.total_cycles++;
    
    // Note: Instructions are counted in WriteBack() stage, not here
    // This check was incorrect as it checked mem_wb after WriteBack() cleared it
    
    std::cout << "Cycle: " << cycle_s_ << " PC: " << std::hex << program_counter_ << std::dec;
    if (pipeline_stall_) {
      std::cout << " [STALLED]";
    }
    std::cout << std::endl;
  }
  
  // Calculate CPI
  if (stats_.instructions_retired > 0) {
    stats_.cpi = static_cast<float>(stats_.total_cycles) / static_cast<float>(stats_.instructions_retired);
  }
  
  if (program_counter_ >= program_size_ && !HasInstructionsInPipeline()) {
    std::cout << "VM_PROGRAM_END" << std::endl;
    std::cout << "Statistics: Cycles=" << stats_.total_cycles 
              << " Instructions=" << stats_.instructions_retired
              << " CPI=" << stats_.cpi
              << " Stalls=" << stats_.stalls
              << " Data Hazards=" << stats_.data_hazards;
    if (forwarding_enabled_) {
      std::cout << " Forwarding Events=" << stats_.forwarding_events;
    }
    if (branch_prediction_enabled_) {
      // Calculate prediction accuracy
      if (stats_.branch_predictions > 0) {
        stats_.branch_prediction_accuracy = 
          100.0f * (1.0f - static_cast<float>(stats_.branch_mispredictions) / 
                          static_cast<float>(stats_.branch_predictions));
      }
      std::cout << " Branch Predictions=" << stats_.branch_predictions
                << " Mispredictions=" << stats_.branch_mispredictions
                << " Accuracy=" << std::fixed << std::setprecision(2) 
                << stats_.branch_prediction_accuracy << "%";
    }
    std::cout << std::endl;
    output_status_ = "VM_PROGRAM_END";
  }
  
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

void RV5SVM::DebugRun() {
  ClearStop();
  // Similar to Run but with breakpoint support
  Run();
}

void RV5SVM::Step() {
  if (HasInstructionsInPipeline()) {
    Tick();
    std::cout << "VM_STEP_COMPLETED" << std::endl;
    output_status_ = "VM_STEP_COMPLETED";
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
  } else {
    std::cout << "VM_PROGRAM_END" << std::endl;
    output_status_ = "VM_PROGRAM_END";
  }
}

void RV5SVM::Undo() {
  // Placeholder for undo functionality
}

void RV5SVM::Redo() {
  // Placeholder for redo functionality
}

void RV5SVM::SetHazardDetectionEnabled(bool enabled) {
  hazard_detection_enabled_ = enabled;
}

bool RV5SVM::IsHazardDetectionEnabled() const {
  return hazard_detection_enabled_;
}

void RV5SVM::SetForwardingEnabled(bool enabled) {
  forwarding_enabled_ = enabled;
}

bool RV5SVM::IsForwardingEnabled() const {
  return forwarding_enabled_;
}

void RV5SVM::SetBranchPredictionEnabled(bool enabled) {
  branch_prediction_enabled_ = enabled;
}

bool RV5SVM::IsBranchPredictionEnabled() const {
  return branch_prediction_enabled_;
}

void RV5SVM::SetBranchPredictionPolicy(BranchPredictor::PredictionPolicy policy) {
  branch_predictor_.SetPolicy(policy);
}

void RV5SVM::Reset() {
  program_counter_ = 0;
  if_id.Clear();
  id_ex.Clear();
  ex_mem.Clear();
  mem_wb.Clear();
  registers_.Reset();
  pipeline_stall_ = false;
  pipeline_flush_ = false;
  cycle_s_ = 0;
  instructions_retired_ = 0;
  stats_ = PipelineStats();  // Reset statistics
}

