/**
 * @file rv5s_control_unit.h
 * @brief RV5S Control Unit for 5-stage pipeline
 */

#ifndef RV5S_CONTROL_UNIT_H
#define RV5S_CONTROL_UNIT_H

#include "../control_unit_base.h"

/**
 * @brief RV5S Control Unit
 * Extends base control unit for pipeline-specific needs
 */
class RV5SControlUnit : public ControlUnit {
public:
    void SetControlSignals(uint32_t instruction) override;
    alu::AluOp GetAluSignal(uint32_t instruction, bool ALUOp) override;
    
    // Additional control signals for pipeline
    bool GetPCSrc() const;
    void SetPCSrc(bool value);
    
private:
    bool pc_src_ = false;
};

#endif // RV5S_CONTROL_UNIT_H
