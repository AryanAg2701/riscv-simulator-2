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

class HazardDetectionUnit {
public:

    bool DetectDataHazard(const IF_ID_Register& if_id,
                         const ID_EX_Register& id_ex,
                         const EX_MEM_Register& ex_mem,
                         const MEM_WB_Register& mem_wb);
    
    bool DetectLoadUseHazard(const IF_ID_Register& if_id,
                            const EX_MEM_Register& ex_mem);
    
    bool ShouldStall(const IF_ID_Register& if_id,
                    const ID_EX_Register& id_ex,
                    const EX_MEM_Register& ex_mem,
                    const MEM_WB_Register& mem_wb);
    
    static bool WillWriteRegister(bool reg_write, uint8_t rd);
    
    static void ExtractRegisters(uint32_t instruction, uint8_t& rs1, uint8_t& rs2, uint8_t& rd);
};

#endif // HAZARD_DETECTION_UNIT_H

