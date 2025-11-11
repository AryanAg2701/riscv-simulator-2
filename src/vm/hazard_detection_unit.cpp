/**
 * @file hazard_detection_unit.cpp
 * @brief Hazard Detection Unit implementation
 */

#include "vm/hazard_detection_unit.h"
#include <cstdint>

bool HazardDetectionUnit::WillWriteRegister(bool reg_write, uint8_t rd) {
    // x0 (register 0) is always 0, never written
    return reg_write && (rd != 0);
}

void HazardDetectionUnit::ExtractRegisters(uint32_t instruction, uint8_t& rs1, uint8_t& rs2, uint8_t& rd) {
    rs1 = (instruction >> 15) & 0b11111;
    rs2 = (instruction >> 20) & 0b11111;
    rd = (instruction >> 7) & 0b11111;
}

bool HazardDetectionUnit::DetectDataHazard(const IF_ID_Register& if_id,
                                           const ID_EX_Register& id_ex,
                                           const EX_MEM_Register& ex_mem,
                                           const MEM_WB_Register& mem_wb) {
    if (!if_id.valid) {
        return false;  // No instruction in ID stage
    }
    
    uint32_t instruction = if_id.instruction;
    uint8_t id_rs1, id_rs2, id_rd;
    ExtractRegisters(instruction, id_rs1, id_rs2, id_rd);

    // Check EX/MEM stage
    // Note: ex_mem.valid check is implicit - if not valid, reg_write will be false
    if (ex_mem.valid && WillWriteRegister(ex_mem.reg_write, ex_mem.rd)) {
        // Check if ID stage reads from the register that EX/MEM is writing to
        // x0 (register 0) is always 0, so no hazard if reading from x0
        if ((id_rs1 != 0 && id_rs1 == ex_mem.rd) || (id_rs2 != 0 && id_rs2 == ex_mem.rd)) {
            return true;
        }
    }
    
    // Check MEM/WB stage
    // Note: mem_wb.valid check is implicit - if not valid, reg_write will be false
    if (mem_wb.valid && WillWriteRegister(mem_wb.reg_write, mem_wb.rd)) {
        // Check if ID stage reads from the register that MEM/WB is writing to
        // x0 (register 0) is always 0, so no hazard if reading from x0
        if ((id_rs1 != 0 && id_rs1 == mem_wb.rd) || (id_rs2 != 0 && id_rs2 == mem_wb.rd)) {
            return true;
        }
    }
    
    return false;  // No hazard
}

bool HazardDetectionUnit::DetectLoadUseHazard(const IF_ID_Register& if_id,
                                              const MEM_WB_Register& mem_wb) {
    if (!if_id.valid || !mem_wb.valid) {
        return false;
    }
    
    // Load use hazard: Check if instruction that was in MEM stage is a load
    // and the instruction in ID stage needs that register
    // 
    // IMPORTANT: When Decode() runs, stages execute in reverse order:
    // 1. MemoryAccess() has already run - processed ex_mem (load in MEM), wrote to mem_wb
    // 2. Execute() has already run - processed id_ex, wrote to ex_mem
    // 3. Decode() is now running
    //
    // So the load instruction that was in MEM stage is now in mem_wb.
    // We check mem_wb to see if it was a load instruction.
    // Load instructions have opcode 0b0000011
    //
    // Note: Load-use hazard requires a stall because:
    // - Load data becomes available at the END of MEM stage
    // - But we're checking in ID stage (before EX stage)
    // - Even with forwarding, we can't forward from EX/MEM for loads (data not ready)
    // - We can forward from MEM/WB, but that's the NEXT cycle
    // - So we need 1 cycle stall to let the load complete
    uint32_t mem_wb_instruction = mem_wb.instruction;
    uint8_t opcode = mem_wb_instruction & 0x7F;  // Lower 7 bits
    
    // Check if mem_wb contains a load instruction (opcode 0b0000011)
    bool is_load = (opcode == 0b0000011);
    
    if (is_load && WillWriteRegister(mem_wb.reg_write, mem_wb.rd)) {
        // Extract registers from ID stage instruction
        uint32_t instruction = if_id.instruction;
        uint8_t id_rs1, id_rs2, id_rd;
        ExtractRegisters(instruction, id_rs1, id_rs2, id_rd);
        
        // Check if ID stage reads from the register that LOAD is writing to
        // Also check that we're not reading from x0 (register 0, always 0)
        if ((id_rs1 != 0 && id_rs1 == mem_wb.rd) || (id_rs2 != 0 && id_rs2 == mem_wb.rd)) {
            return true; 
        }
    }
    
    return false;
}

bool HazardDetectionUnit::ShouldStall(const IF_ID_Register& if_id,
                                     const ID_EX_Register& id_ex,
                                     const EX_MEM_Register& ex_mem,
                                     const MEM_WB_Register& mem_wb) {
    // Check load-use hazard first (more critical, needs immediate stall)
    // When Decode() runs, MemoryAccess() has already run, so the load that
    // was in MEM stage is now in mem_wb. We check mem_wb to detect load-use hazards.
    if (DetectLoadUseHazard(if_id, mem_wb)) {
        return true;
    }
    
    if (DetectDataHazard(if_id, id_ex, ex_mem, mem_wb)) {
        return true;
    }
    
    return false;  // No stall needed
}

