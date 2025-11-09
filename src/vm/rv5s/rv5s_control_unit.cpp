/**
 * @file rv5s_control_unit.cpp
 * @brief RV5S Control Unit implementation
 */

#include "vm/rv5s/rv5s_control_unit.h"
#include "vm/alu.h"

#include <cstdint>

#include "common/instructions.h"
using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

void RV5SControlUnit::SetControlSignals(uint32_t instruction) {
  uint8_t opcode = instruction & 0b1111111;

  alu_src_ = mem_to_reg_ = reg_write_ = mem_read_ = mem_write_ = branch_ = false;
  alu_op_ = false;
  pc_src_ = false;

  switch (opcode) {
    case 0b0110011:{
      reg_write_ = true;
      alu_op_ = true;
      break;
    }
    case 0b0000011: {
      alu_src_ = true;
      mem_to_reg_ = true;
      reg_write_ = true;
      mem_read_ = true;
      break;
    }
    case 0b0100011: {
      alu_src_ = true;
      alu_op_ = true;
      mem_write_ = true;
      break;
    }
    case 0b1100011: {
      alu_op_ = true;
      branch_ = true;
      break;
    }
    case 0b0010011: {
      alu_src_ = true;
      reg_write_ = true;
      alu_op_ = true;
      break;
    }
    case 0b0110111: {// LUI
      alu_src_ = true;
      reg_write_ = true;
      alu_op_ = true;
      break;
    }
    case 0b0010111: {//upper immediat wala
      alu_src_ = true;
      reg_write_ = true;
      alu_op_ = true;
      break;
    }
    case 0b1101111: { //jal
      reg_write_ = true;
      branch_ = true;
      pc_src_ = true;
      break;
    }
    case 0b1100111: {// jalr
      alu_src_ = true;
      reg_write_ = true;
      branch_ = true;
      pc_src_ = true;
      break;
    }
    case 0b0000001: {
      reg_write_ = true;
      alu_op_ = true;
      break;
    }
  //   // F extension + D extension
  //   case 0b0000111: {
  //     alu_src_ = true;
  //     mem_to_reg_ = true;
  //     reg_write_ = true;
  //     mem_read_ = true;
  //     break;
  //   }
  //   case 0b0100111: {
  //     alu_src_ = true;
  //     alu_op_ = true;
  //     mem_write_ = true;
  //     break;
  //   }
  //   case 0b1010011: {
  //     alu_op_ = true;
  //     break;
  //   }
  //   default:
  //     break;
  }
}

alu::AluOp RV5SControlUnit::GetAluSignal(uint32_t instruction, bool ALUOp) {
    (void)ALUOp;
    uint8_t opcode = instruction & 0b1111111; 
    uint8_t funct3 = (instruction >> 12) & 0b111;
    uint8_t funct7 = (instruction >> 25) & 0b1111111;
    uint8_t funct5 = (instruction >> 20) & 0b11111;
    uint8_t funct2 = (instruction >> 25) & 0b11;

    // Use same ALU signal generation as RVSS
    // This is a simplified version - can be expanded later
    switch (opcode) {
    case 0b0110011: {// R-Type
        switch (funct3) {
        case 0b000: {
            switch (funct7) {
            case 0x0000000: return alu::AluOp::kAdd;
            case 0b0100000: return alu::AluOp::kSub;
            case 0b0000001: return alu::AluOp::kMul;
            }
            break;
        }
        case 0b001: {
            if (funct7 == 0b0000000) return alu::AluOp::kSll;
            if (funct7 == 0b0000001) return alu::AluOp::kMulh;
            break;
        }
        case 0b100: {
            if (funct7 == 0b0000000) return alu::AluOp::kXor;
            if (funct7 == 0b0000001) return alu::AluOp::kDiv;
            break;
        }
        case 0b101: {
            if (funct7 == 0b0000000) return alu::AluOp::kSrl;
            if (funct7 == 0b0100000) return alu::AluOp::kSra;
            if (funct7 == 0b0000001) return alu::AluOp::kDivu;
            break;
        }
        case 0b110: {
            if (funct7 == 0b0000000) return alu::AluOp::kOr;
            if (funct7 == 0b0000001) return alu::AluOp::kRem;
            break;
        }
        case 0b111: {
            if (funct7 == 0b0000000) return alu::AluOp::kAnd;
            if (funct7 == 0b0000001) return alu::AluOp::kRemu;
            break;
        }
        }
        break;
    }
    case 0b0010011: {// I-Type
        switch (funct3) {
        case 0b000: return alu::AluOp::kAdd;
        case 0b001: return alu::AluOp::kSll;
        case 0b010: return alu::AluOp::kSlt;
        case 0b011: return alu::AluOp::kSltu;
        case 0b100: return alu::AluOp::kXor;
        case 0b101: {
            if (funct7 == 0b0000000) return alu::AluOp::kSrl;
            if (funct7 == 0b0100000) return alu::AluOp::kSra;
            break;
        }
        case 0b110: return alu::AluOp::kOr;
        case 0b111: return alu::AluOp::kAnd;
        }
        break;
    }
    case 0b1100011: {// B-Type
        switch (funct3) {
        case 0b000: // BEQ
        case 0b001: // BNE
            return alu::AluOp::kSub;
        case 0b100: // BLT
        case 0b101: // BGE
            return alu::AluOp::kSlt;
        case 0b110: // BLTU
        case 0b111: // BGEU
            return alu::AluOp::kSltu;
        }
        break;
    }
    case 0b0000011: // Load
    case 0b0100011: // Store
    case 0b1100111: // JALR
    case 0b1101111: // JAL
    case 0b0110111: // LUI
    case 0b0010111: // AUIPC
        return alu::AluOp::kAdd;
    }
    
    return alu::AluOp::kNone;
}

bool RV5SControlUnit::GetPCSrc() const {
    return pc_src_;
}

void RV5SControlUnit::SetPCSrc(bool value) {
    pc_src_ = value;
}

