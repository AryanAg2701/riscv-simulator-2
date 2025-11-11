/**
 * @file branch_predictor.cpp
 * @brief Static Branch Predictor implementation
 */

#include "vm/branch_predictor.h"
#include <cstdint>

BranchPredictor::BranchPredictor(PredictionPolicy policy) 
    : policy_(policy) {
}

void BranchPredictor::SetPolicy(PredictionPolicy policy) {
    policy_ = policy;
}

BranchPredictor::PredictionPolicy BranchPredictor::GetPolicy() const {
    return policy_;
}

bool BranchPredictor::IsBranchOrJump(uint32_t instruction) {
    uint8_t opcode = instruction & 0b1111111;
    return opcode == 0b1100011 ||  // B-type (conditional branches)
           opcode == 0b1101111 ||  // JAL
           opcode == 0b1100111;    // JALR
}

bool BranchPredictor::IsConditionalBranch(uint32_t instruction) {
    uint8_t opcode = instruction & 0b1111111;
    return opcode == 0b1100011;  // B-type (BEQ, BNE, BLT, BGE, BLTU, BGEU)
}

bool BranchPredictor::IsUnconditionalJump(uint32_t instruction) {
    uint8_t opcode = instruction & 0b1111111;
    return opcode == 0b1101111 ||  // JAL
           opcode == 0b1100111;    // JALR
}

int32_t BranchPredictor::ExtractBTypeImmediate(uint32_t instruction) {
    // B-type immediate format: [12|10:5|4:1|11]
    // Bits: imm[12] imm[10:5] rs2 rs1 funct3 imm[4:1] imm[11] opcode
    int32_t imm = 0;
    imm |= ((instruction >> 31) & 0x1) << 12;  // imm[12]
    imm |= ((instruction >> 25) & 0x3F) << 5;  // imm[10:5]
    imm |= ((instruction >> 8) & 0xF) << 1;   // imm[4:1]
    imm |= ((instruction >> 7) & 0x1) << 11;  // imm[11]
    
    // Sign extend from bit 12
    if (imm & 0x1000) {
        imm |= 0xFFFFE000;  // Sign extend
    }
    
    return imm;
}

int32_t BranchPredictor::ExtractJTypeImmediate(uint32_t instruction) {
    // J-type immediate format: [20|10:1|11|19:12]
    // Bits: imm[20] imm[10:1] imm[11] imm[19:12] rd opcode
    int32_t imm = 0;
    imm |= ((instruction >> 31) & 0x1) << 20;  // imm[20]
    imm |= ((instruction >> 21) & 0x3FF) << 1; // imm[10:1]
    imm |= ((instruction >> 20) & 0x1) << 11;  // imm[11]
    imm |= ((instruction >> 12) & 0xFF) << 12; // imm[19:12]
    
    // Sign extend from bit 20
    if (imm & 0x100000) {
        imm |= 0xFFE00000;  // Sign extend
    }
    
    return imm;
}

bool BranchPredictor::Predict(uint32_t instruction, uint64_t pc) const {
    (void)pc;  // Not used in static prediction
    
    // Unconditional jumps are always taken
    if (IsUnconditionalJump(instruction)) {
        return true;
    }
    
    // Conditional branches use static policy
    if (IsConditionalBranch(instruction)) {
        return (policy_ == ALWAYS_TAKEN);
    }
    
    // Not a branch/jump
    return false;
}

uint64_t BranchPredictor::CalculateBranchTarget(uint32_t instruction, uint64_t pc) const {
    uint8_t opcode = instruction & 0b1111111;
    
    if (opcode == 0b1100011) {
        // B-type: PC + sign_extended(imm)
        int32_t imm = ExtractBTypeImmediate(instruction);
        return static_cast<uint64_t>(static_cast<int64_t>(pc) + imm);
    } else if (opcode == 0b1101111) {
        // JAL: PC + sign_extended(imm)
        int32_t imm = ExtractJTypeImmediate(instruction);
        return static_cast<uint64_t>(static_cast<int64_t>(pc) + imm);
    } else if (opcode == 0b1100111) {
        // JALR: rs1 + sign_extended(imm)
        // Cannot calculate without rs1 value, return 0 as sentinel
        // Caller should handle JALR separately
        return 0;
    }
    
    return pc + 4;  // Default: sequential
}

