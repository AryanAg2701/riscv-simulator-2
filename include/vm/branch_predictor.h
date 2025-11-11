/**
 * @file branch_predictor.h
 * @brief Branch Predictor for 5-stage pipeline (Mode 5 & Mode 6)
 * 
 * Implements branch prediction with multiple policies:
 * - Mode 5: Static prediction (Always Taken / Always Not Taken)
 * - Mode 6: Dynamic 1-bit prediction (learns from branch outcomes)
 */

#ifndef BRANCH_PREDICTOR_H
#define BRANCH_PREDICTOR_H

#include <cstdint>
#include <unordered_map>

/**
 * @brief Branch Predictor
 * 
 * Provides static and dynamic branch prediction for branches and jumps.
 * Unconditional jumps (JAL/JALR) are always predicted as taken.
 */
class BranchPredictor {
public:
    /**
     * @brief Branch prediction policy
     */
    enum PredictionPolicy {
        ALWAYS_TAKEN,      ///< Mode 5: Predict all branches as taken
        ALWAYS_NOT_TAKEN,  ///< Mode 5: Predict all branches as not taken
        DYNAMIC_1BIT       ///< Mode 6: Dynamic 1-bit prediction (learns from outcomes)
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
     * For conditional branches:
     *   - Static mode: uses static policy (ALWAYS_TAKEN/ALWAYS_NOT_TAKEN)
     *   - Dynamic mode: uses Branch Prediction Table (BPT) indexed by PC
     * For unconditional jumps (JAL/JALR), always predicts taken.
     * 
     * @param instruction Instruction to predict
     * @param pc Program counter of the instruction
     * @return true if branch is predicted taken, false otherwise
     */
    bool Predict(uint32_t instruction, uint64_t pc);
    
    /**
     * @brief Update the predictor based on actual branch outcome (Mode 6)
     * 
     * Trains the dynamic predictor by updating the prediction bit for the branch.
     * For 1-bit prediction:
     *   - If branch was taken, set prediction to taken (1)
     *   - If branch was not taken, set prediction to not taken (0)
     * 
     * Only updates when policy is DYNAMIC_1BIT.
     * 
     * @param pc Program counter of the branch instruction
     * @param actual_taken True if branch was actually taken, false otherwise
     */
    void Update(uint64_t pc, bool actual_taken);
    
    /**
     * @brief Reset/Clear the prediction table (Mode 6)
     * 
     * Clears all entries from the Branch Prediction Table.
     * Useful when resetting the VM or switching programs.
     */
    void Reset();
    
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
    
    // Dynamic 1-bit prediction table (Mode 6)
    // Maps PC -> prediction bit (true = taken, false = not taken)
    // For new branches, default to not taken (false)
    mutable std::unordered_map<uint64_t, bool> prediction_table_;
    
    /**
     * @brief Get prediction from BPT for dynamic mode
     * @param pc Program counter to look up
     * @return Prediction bit (true = taken, false = not taken)
     */
    bool GetDynamicPrediction(uint64_t pc) const;
};

#endif // BRANCH_PREDICTOR_H

