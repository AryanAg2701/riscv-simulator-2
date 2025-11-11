/**
 * @file branch_predictor.h
 * @brief Static Branch Predictor for 5-stage pipeline (Mode 5)
 * 
 * Implements static branch prediction with fixed policies:
 * - Always Taken: Predict all conditional branches as taken
 * - Always Not Taken: Predict all conditional branches as not taken
 */

#ifndef BRANCH_PREDICTOR_H
#define BRANCH_PREDICTOR_H

#include <cstdint>

/**
 * @brief Static Branch Predictor
 * 
 * Provides static branch prediction for conditional branches.
 * Unconditional jumps (JAL/JALR) are always predicted as taken.
 */
class BranchPredictor {
public:
    /**
     * @brief Branch prediction policy
     */
    enum PredictionPolicy {
        ALWAYS_TAKEN,      ///< Predict all branches as taken
        ALWAYS_NOT_TAKEN   ///< Predict all branches as not taken
    };
    
    /**
     * @brief Constructor
     * @param policy Prediction policy to use
     */
    explicit BranchPredictor(PredictionPolicy policy = ALWAYS_NOT_TAKEN);
    
    /**
     * @brief Set prediction policy
     * @param policy New prediction policy
     */
    void SetPolicy(PredictionPolicy policy);
    
    /**
     * @brief Get current prediction policy
     * @return Current prediction policy
     */
    PredictionPolicy GetPolicy() const;
    
    /**
     * @brief Predict if a branch will be taken
     * 
     * For conditional branches, uses static policy.
     * For unconditional jumps (JAL/JALR), always predicts taken.
     * 
     * @param instruction Instruction to predict
     * @param pc Program counter of the instruction
     * @return true if branch is predicted taken, false otherwise
     */
    bool Predict(uint32_t instruction, uint64_t pc) const;
    
    /**
     * @brief Calculate branch target address
     * 
     * Calculates the target address for a branch instruction.
     * For conditional branches: PC + sign_extended(imm)
     * For JAL: PC + sign_extended(imm)
     * For JALR: Requires rs1 value (returns 0, should be handled separately)
     * 
     * @param instruction Branch instruction
     * @param pc Program counter of the instruction
     * @return Calculated target address (0 for JALR, which needs rs1)
     */
    uint64_t CalculateBranchTarget(uint32_t instruction, uint64_t pc) const;
    
    /**
     * @brief Check if instruction is a branch or jump
     * @param instruction Instruction to check
     * @return true if instruction is a branch or jump
     */
    static bool IsBranchOrJump(uint32_t instruction);
    
    /**
     * @brief Check if instruction is a conditional branch (BEQ, BNE, etc.)
     * @param instruction Instruction to check
     * @return true if instruction is a conditional branch
     */
    static bool IsConditionalBranch(uint32_t instruction);
    
    /**
     * @brief Check if instruction is an unconditional jump (JAL/JALR)
     * @param instruction Instruction to check
     * @return true if instruction is JAL or JALR
     */
    static bool IsUnconditionalJump(uint32_t instruction);
    
    /**
     * @brief Extract and sign-extend immediate from B-type instruction
     * @param instruction B-type instruction
     * @return Sign-extended immediate value
     */
    static int32_t ExtractBTypeImmediate(uint32_t instruction);
    
    /**
     * @brief Extract and sign-extend immediate from J-type instruction
     * @param instruction J-type instruction
     * @return Sign-extended immediate value
     */
    static int32_t ExtractJTypeImmediate(uint32_t instruction);
    
private:
    PredictionPolicy policy_;  ///< Current prediction policy
};

#endif // BRANCH_PREDICTOR_H

