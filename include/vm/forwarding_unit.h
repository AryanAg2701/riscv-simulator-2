/**
 * @file forwarding_unit.h
 * @brief Forwarding Unit for 5-stage pipeline (Mode 4)
 * 
 * Implements data forwarding to minimize pipeline stalls by forwarding
 * results from EX/MEM and MEM/WB stages directly to the Execute stage.
 */

#ifndef FORWARDING_UNIT_H
#define FORWARDING_UNIT_H

#include "vm/pipeline_registers.h"
#include <cstdint>
#include <utility>

/**
 * @brief Forwarding Unit for 5-stage pipeline
 * 
 * Detects when data can be forwarded from later pipeline stages
 * to avoid stalls and improve performance.
 */
class ForwardingUnit {
public:
    /**
     * @brief Forwarding control signals for ALU operands
     */
    enum ForwardSignal {
        FORWARD_NONE,      ///< No forwarding, use register file value
        FORWARD_EX_MEM,    ///< Forward from EX/MEM stage
        FORWARD_MEM_WB     ///< Forward from MEM/WB stage
    };
    
    /**
     * @brief Determine forwarding signals for rs1 (ForwardA) and rs2 (ForwardB)
     * 
     * Priority: EX/MEM > MEM/WB (more recent data takes precedence)
     * 
     * @param id_ex ID/EX pipeline register
     * @param ex_mem EX/MEM pipeline register
     * @param mem_wb MEM/WB pipeline register
     * @return Pair of (ForwardSignal, ForwardSignal) signals for (rs1, rs2)
     */
    std::pair<ForwardSignal, ForwardSignal> GetForwardingSignals(
        const ID_EX_Register& id_ex,
        const EX_MEM_Register& ex_mem,
        const MEM_WB_Register& mem_wb) const;
    
    /**
     * @brief Get forwarded value for rs1 (operand A)
     * 
     * @param forward Forwarding signal for rs1
     * @param id_ex ID/EX pipeline register
     * @param ex_mem EX/MEM pipeline register
     * @param mem_wb MEM/WB pipeline register
     * @return Forwarded value for rs1
     */
    uint64_t GetForwardedValueA(
        ForwardSignal forward,
        const ID_EX_Register& id_ex,
        const EX_MEM_Register& ex_mem,
        const MEM_WB_Register& mem_wb) const;
    
    /**
     * @brief Get forwarded value for rs2 (operand B)
     * 
     * @param forward Forwarding signal for rs2
     * @param id_ex ID/EX pipeline register
     * @param ex_mem EX/MEM pipeline register
     * @param mem_wb MEM/WB pipeline register
     * @return Forwarded value for rs2
     */
    uint64_t GetForwardedValueB(
        ForwardSignal forward,
        const ID_EX_Register& id_ex,
        const EX_MEM_Register& ex_mem,
        const MEM_WB_Register& mem_wb) const;
    
    /**
     * @brief Get forwarded value for store data (rs2 in memory stage)
     * 
     * @param forward Forwarding signal for rs2
     * @param id_ex ID/EX pipeline register (contains original rs2)
     * @param ex_mem EX/MEM pipeline register
     * @param mem_wb MEM/WB pipeline register
     * @return Forwarded value for store data
     */
    uint64_t GetForwardedStoreData(
        ForwardSignal forward,
        const ID_EX_Register& id_ex,
        const EX_MEM_Register& ex_mem,
        const MEM_WB_Register& mem_wb) const;
    
    /**
     * @brief Check if a register will be written by a pipeline stage
     * 
     * @param reg_write Register write enable signal
     * @param rd Destination register
     * @return true if register will be written
     */
    static bool WillWriteRegister(bool reg_write, uint8_t rd);
    
private:
    /**
     * @brief Get the value to forward from EX/MEM stage
     * 
     * For loads, we can't forward (data not ready until end of MEM stage)
     * For ALU operations, forward alu_result
     */
    uint64_t GetEXMEMValue(const EX_MEM_Register& ex_mem) const;
    
    /**
     * @brief Get the value to forward from MEM/WB stage
     * 
     * Forward either mem_read_data (for loads) or alu_result (for ALU ops)
     */
    uint64_t GetMEMWBValue(const MEM_WB_Register& mem_wb) const;
};

#endif // FORWARDING_UNIT_H

