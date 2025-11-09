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
    if (WillWriteRegister(ex_mem.reg_write, ex_mem.rd)) {
        if (id_rs1 == ex_mem.rd || id_rs2 == ex_mem.rd) {
            return true;
        }
    }
    
    // Check MEM/WB stage
    if (WillWriteRegister(mem_wb.reg_write, mem_wb.rd)) {
        if (id_rs1 == mem_wb.rd || id_rs2 == mem_wb.rd) {
            return true;
        }
    }
    
    return false;  // No hazard
}

bool HazardDetectionUnit::DetectLoadUseHazard(const IF_ID_Register& if_id,
                                              const EX_MEM_Register& ex_mem) {
    if (!if_id.valid || !ex_mem.valid) {
        return false;
    }
    
    // Load use hazard
    if (ex_mem.mem_read && WillWriteRegister(ex_mem.reg_write, ex_mem.rd)) {
        // Extract registers from ID stage instruction
        uint32_t instruction = if_id.instruction;
        uint8_t id_rs1, id_rs2, id_rd;
        ExtractRegisters(instruction, id_rs1, id_rs2, id_rd);
        
        // Check if ID stage reads from the register that LOAD is writing to
        if (id_rs1 == ex_mem.rd || id_rs2 == ex_mem.rd) {
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
    if (DetectLoadUseHazard(if_id, ex_mem)) {
        return true;
    }
    
    if (DetectDataHazard(if_id, id_ex, ex_mem, mem_wb)) {
        return true;
    }
    
    return false;  // No stall needed
}

