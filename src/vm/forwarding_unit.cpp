/**
 * @file forwarding_unit.cpp
 * @brief Forwarding Unit implementation
 */

#include "vm/forwarding_unit.h"
#include <cstdint>

bool ForwardingUnit::WillWriteRegister(bool reg_write, uint8_t rd) {
    // x0 (register 0) is always 0, never written
    return reg_write && (rd != 0);
}

std::pair<ForwardingUnit::ForwardSignal, ForwardingUnit::ForwardSignal> 
ForwardingUnit::GetForwardingSignals(
    const ID_EX_Register& id_ex,
    const EX_MEM_Register& ex_mem,
    const MEM_WB_Register& mem_wb) const {
    
    ForwardSignal forwardA = FORWARD_NONE;
    ForwardSignal forwardB = FORWARD_NONE;
    
    // Check forwarding for rs1 (ForwardA)
    // Priority: EX/MEM > MEM/WB
    
    // Check EX/MEM forwarding for rs1
    if (ex_mem.valid && WillWriteRegister(ex_mem.reg_write, ex_mem.rd)) {
        if (id_ex.rs1 == ex_mem.rd) {
            // Can forward from EX/MEM, but check if it's a load
            // For loads, we can't forward immediately (data not ready)
            if (!ex_mem.mem_read) {
                forwardA = FORWARD_EX_MEM;
            }
            // If it's a load, we can't forward, so check MEM/WB below
        }
    }
    
    // Check MEM/WB forwarding for rs1 (only if EX/MEM didn't forward)
    if (forwardA == FORWARD_NONE && mem_wb.valid && WillWriteRegister(mem_wb.reg_write, mem_wb.rd)) {
        if (id_ex.rs1 == mem_wb.rd) {
            forwardA = FORWARD_MEM_WB;
        }
    }
    
    // Check forwarding for rs2 (ForwardB)
    // Priority: EX/MEM > MEM/WB
    
    // Check EX/MEM forwarding for rs2
    if (ex_mem.valid && WillWriteRegister(ex_mem.reg_write, ex_mem.rd)) {
        if (id_ex.rs2 == ex_mem.rd) {
            // Can forward from EX/MEM, but check if it's a load
            if (!ex_mem.mem_read) {
                forwardB = FORWARD_EX_MEM;
            }
        }
    }
    
    // Check MEM/WB forwarding for rs2 (only if EX/MEM didn't forward)
    if (forwardB == FORWARD_NONE && mem_wb.valid && WillWriteRegister(mem_wb.reg_write, mem_wb.rd)) {
        if (id_ex.rs2 == mem_wb.rd) {
            forwardB = FORWARD_MEM_WB;
        }
    }
    
    return std::make_pair(forwardA, forwardB);
}

uint64_t ForwardingUnit::GetForwardedValueA(
    ForwardSignal forward,
    const ID_EX_Register& id_ex,
    const EX_MEM_Register& ex_mem,
    const MEM_WB_Register& mem_wb) const {
    
    switch (forward) {
        case FORWARD_EX_MEM:
            return GetEXMEMValue(ex_mem);
        case FORWARD_MEM_WB:
            return GetMEMWBValue(mem_wb);
        case FORWARD_NONE:
        default:
            return id_ex.read_data1;  // Use register file value
    }
}

uint64_t ForwardingUnit::GetForwardedValueB(
    ForwardSignal forward,
    const ID_EX_Register& id_ex,
    const EX_MEM_Register& ex_mem,
    const MEM_WB_Register& mem_wb) const {
    
    switch (forward) {
        case FORWARD_EX_MEM:
            return GetEXMEMValue(ex_mem);
        case FORWARD_MEM_WB:
            return GetMEMWBValue(mem_wb);
        case FORWARD_NONE:
        default:
            return id_ex.read_data2;  // Use register file value
    }
}

uint64_t ForwardingUnit::GetForwardedStoreData(
    ForwardSignal forward,
    const ID_EX_Register& id_ex,
    const EX_MEM_Register& ex_mem,
    const MEM_WB_Register& mem_wb) const {
    
    switch (forward) {
        case FORWARD_EX_MEM:
            return GetEXMEMValue(ex_mem);
        case FORWARD_MEM_WB:
            return GetMEMWBValue(mem_wb);
        case FORWARD_NONE:
        default:
            return id_ex.read_data2;  // Use register file value
    }
}

uint64_t ForwardingUnit::GetEXMEMValue(const EX_MEM_Register& ex_mem) const {
    // For EX/MEM, we forward the ALU result
    // Note: For loads, we can't forward from EX/MEM (data not ready until end of MEM)
    return ex_mem.alu_result;
}

uint64_t ForwardingUnit::GetMEMWBValue(const MEM_WB_Register& mem_wb) const {
    // For MEM/WB, forward the final result:
    // - mem_read_data if it was a load (mem_to_reg = true)
    // - alu_result if it was an ALU operation (mem_to_reg = false)
    if (mem_wb.mem_to_reg) {
        return mem_wb.mem_read_data;
    } else {
        return mem_wb.alu_result;
    }
}

