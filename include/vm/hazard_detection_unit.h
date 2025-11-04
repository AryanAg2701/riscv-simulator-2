/**
 * @file hazard_detection_unit.h
 * @brief Hazard Detection Unit for 5-stage pipeline
 * 
 * Detects data hazards (RAW) and load-use hazards, indicating when
 * pipeline stalls are needed to ensure correct execution.
 */

#ifndef HAZARD_DETECTION_UNIT_H
#define HAZARD_DETECTION_UNIT_H

#include "vm/pipeline_registers.h"
#include <cstdint>

/**
 * @brief Hazard Detection Unit
 * 
 * Detects pipeline hazards and determines when stalls are needed.
 * Mode 3: Detects hazards and signals stalls (no forwarding).
 */
class HazardDetectionUnit {
public:
    /**
     * @brief Detects if a data hazard (RAW) exists
     * 
     * Checks if instruction in ID stage reads from a register that
     * is being written by instructions in EX/MEM or MEM/WB stages.
     * 
     * @param id_ex Instruction in ID stage (about to enter EX)
     * @param ex_mem Instruction in EX/MEM stage
     * @param mem_wb Instruction in MEM/WB stage
     * @return true if RAW hazard detected, false otherwise
     */
    bool DetectDataHazard(const IF_ID_Register& if_id,
                         const ID_EX_Register& id_ex,
                         const EX_MEM_Register& ex_mem,
                         const MEM_WB_Register& mem_wb);
    
    /**
     * @brief Detects load-use hazard
     * 
     * Special case: LOAD instruction in EX stage, and instruction
     * in ID stage needs that data immediately.
     * 
     * @param if_id Instruction in IF/ID stage
     * @param ex_mem Instruction in EX/MEM stage
     * @return true if load-use hazard detected, false otherwise
     */
    bool DetectLoadUseHazard(const IF_ID_Register& if_id,
                            const EX_MEM_Register& ex_mem);
    
    /**
     * @brief Main interface: determines if pipeline should stall
     * 
     * Checks all hazard types and returns true if stall is needed.
     * 
     * @param if_id Instruction in IF/ID stage
     * @param id_ex Instruction in ID/EX stage (if any)
     * @param ex_mem Instruction in EX/MEM stage
     * @param mem_wb Instruction in MEM/WB stage
     * @return true if stall needed, false otherwise
     */
    bool ShouldStall(const IF_ID_Register& if_id,
                    const ID_EX_Register& id_ex,
                    const EX_MEM_Register& ex_mem,
                    const MEM_WB_Register& mem_wb);
    
    /**
     * @brief Check if a register is being written
     * 
     * Helper function to check if a register write will happen.
     * 
     * @param reg_write Control signal indicating register write
     * @param rd Destination register
     * @return true if register will be written, false otherwise
     */
    static bool WillWriteRegister(bool reg_write, uint8_t rd);
    
    /**
     * @brief Extract register fields from instruction
     * 
     * @param instruction Instruction word
     * @param rs1 Output: rs1 register number
     * @param rs2 Output: rs2 register number
     * @param rd Output: rd register number
     */
    static void ExtractRegisters(uint32_t instruction, uint8_t& rs1, uint8_t& rs2, uint8_t& rd);
};

#endif // HAZARD_DETECTION_UNIT_H

