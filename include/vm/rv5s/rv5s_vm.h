/**
 * @file rv5s_vm.h
 * @brief RV5S 5-Stage Pipeline VM definition
 */

#ifndef RV5S_VM_H
#define RV5S_VM_H

#include "vm/vm_base.h"
#include "vm/pipeline_registers.h"
#include "vm/hazard_detection_unit.h"
#include "vm/forwarding_unit.h"
#include "vm/branch_predictor.h"
#include "rv5s_control_unit.h"

#include <stack>
#include <vector>
#include <atomic>
#include <iostream>
#include <cstdint>

/**
 * @brief Step delta for undo/redo functionality
 */
struct RV5SStepDelta {
    uint64_t old_pc = 0;
    uint64_t new_pc = 0;
    IF_ID_Register if_id;
    ID_EX_Register id_ex;
    EX_MEM_Register ex_mem;
    MEM_WB_Register mem_wb;
    std::vector<std::pair<uint8_t, uint64_t>> register_changes;
    std::vector<std::pair<uint64_t, uint64_t>> memory_changes;
};

/**
 * @brief RV5SVM - 5-Stage Pipeline Virtual Machine
 * 
 * Mode 2: Basic 5-stage pipeline (no hazard handling)
 * Instructions execute through pipeline stages concurrently
 */
class RV5SVM : public VmBase {
public:
    // Pipeline registers
    IF_ID_Register if_id;
    ID_EX_Register id_ex;
    EX_MEM_Register ex_mem;
    MEM_WB_Register mem_wb;
    
    // Control unit
    RV5SControlUnit control_unit_;
    
    // Hazard detection unit
    HazardDetectionUnit hazard_unit_;
    
    // Forwarding unit (Mode 4)
    ForwardingUnit forwarding_unit_;
    
    // Branch predictor (Mode 5)
    BranchPredictor branch_predictor_;
    
    // Pipeline control
    bool pipeline_stall_ = false;
    bool pipeline_flush_ = false;
    bool hazard_detection_enabled_ = false;
    bool forwarding_enabled_ = false;
    bool branch_prediction_enabled_ = false;
    
    // Undo/Redo support
    std::stack<RV5SStepDelta> undo_stack_;
    std::stack<RV5SStepDelta> redo_stack_;
    RV5SStepDelta current_delta_;
    
    // Intermediate execution values
    int64_t execution_result_ = 0;
    int64_t memory_result_ = 0;
    uint64_t return_address_ = 0;
    bool branch_flag_ = false;
    int64_t next_pc_ = 0;
    
    // CSR intermediate variables
    uint16_t csr_target_address_ = 0;
    uint64_t csr_old_value_ = 0;
    uint64_t csr_write_val_ = 0;
    uint8_t csr_uimm_ = 0;
    
    // Constructor/Destructor
    RV5SVM();
    ~RV5SVM();

    RV5SVM(const RV5SVM&) = delete;
    RV5SVM& operator=(const RV5SVM&) = delete;
    RV5SVM(RV5SVM&&) = delete;
    RV5SVM& operator=(RV5SVM&&) = delete;
    
    // Pipeline stage methods
    void Fetch();
    void Decode();
    void Execute();
    void MemoryAccess();
    void WriteBack();
    void Tick(); 
    
    // Pipeline control
    void InsertBubble();
    void FlushPipeline();
    bool HasInstructionsInPipeline() const;
    
    // Configuration
    void SetHazardDetectionEnabled(bool enabled);
    bool IsHazardDetectionEnabled() const;
    void SetForwardingEnabled(bool enabled);
    bool IsForwardingEnabled() const;
    void SetBranchPredictionEnabled(bool enabled);
    bool IsBranchPredictionEnabled() const;
    void SetBranchPredictionPolicy(BranchPredictor::PredictionPolicy policy);
    
    // Override base methods
    void Run() override;
    void DebugRun() override;
    void Step() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;
    
    // Helper methods
    void RequestStop();
    bool IsStopRequested() const;
    void ClearStop();
    
    // Statistics
    struct PipelineStats {
        unsigned int total_cycles = 0;
        unsigned int instructions_retired = 0;
        unsigned int stalls = 0;
        unsigned int data_hazards = 0;
        unsigned int control_hazards = 0;
        unsigned int forwarding_events = 0;
        unsigned int branch_predictions = 0;
        unsigned int branch_mispredictions = 0;
        float branch_prediction_accuracy = 0.0f;
        float cpi = 0.0f;
    } stats_;
    
private:
    // Internal helper methods
    void HandleSyscall();
    void ExecuteFloat();
    void ExecuteDouble();
    void ExecuteCsr();
    void WriteMemoryFloat();
    void WriteMemoryDouble();
    void WriteBackFloat();
    void WriteBackDouble();
    void WriteBackCsr();
    
    inline static std::atomic<bool> stop_requested_{false};
};

#endif // RV5S_VM_H
