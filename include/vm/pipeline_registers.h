/**
 * @file pipeline_registers.h
 * @brief Pipeline register structures for 5-stage pipeline
 */

#ifndef PIPELINE_REGISTERS_H
#define PIPELINE_REGISTERS_H

#include <cstdint>

/**
 * @brief IF/ID Pipeline Register
 * Holds instruction and PC from Fetch stage to Decode stage
 */
struct IF_ID_Register {
    uint32_t instruction = 0;
    uint64_t pc = 0;
    bool valid = false;
    
    void Clear() {
        instruction = 0;
        pc = 0;
        valid = false;
    }
};

/**
 * @brief ID/EX Pipeline Register
 * Holds decoded instruction data and control signals
 */
struct ID_EX_Register {
    // Control signals
    bool reg_write = false;
    bool mem_to_reg = false;
    bool branch = false;
    bool mem_read = false;
    bool mem_write = false;
    bool alu_src = false;
    uint8_t alu_op = 0;
    
    // Data
    uint64_t pc = 0;
    uint64_t read_data1 = 0;
    uint64_t read_data2 = 0;
    int32_t imm = 0;
    uint8_t rs1 = 0;
    uint8_t rs2 = 0;
    uint8_t rd = 0;
    uint32_t instruction = 0;
    bool valid = false;
    
    void Clear() {
        reg_write = mem_to_reg = branch = mem_read = mem_write = alu_src = false;
        alu_op = 0;
        pc = read_data1 = read_data2 = 0;
        imm = 0;
        rs1 = rs2 = rd = 0;
        instruction = 0;
        valid = false;
    }
};

/**
 * @brief EX/MEM Pipeline Register
 * Holds execution results and control signals
 */
struct EX_MEM_Register {
    // Control signals
    bool reg_write = false;
    bool mem_to_reg = false;
    bool mem_read = false;
    bool mem_write = false;
    
    // Data
    uint64_t branch_target = 0;
    bool branch_taken = false;
    uint64_t alu_result = 0;
    uint64_t write_data = 0;  // For stores
    uint64_t pc = 0;  // PC of instruction (for passing through to MEM/WB)
    uint8_t rd = 0;
    uint32_t instruction = 0;
    bool valid = false;
    
    void Clear() {
        reg_write = mem_to_reg = mem_read = mem_write = false;
        branch_target = 0;
        branch_taken = false;
        alu_result = write_data = 0;
        pc = 0;
        rd = 0;
        instruction = 0;
        valid = false;
    }
};

/**
 * @brief MEM/WB Pipeline Register
 * Holds memory access results for writeback
 */
struct MEM_WB_Register {
    // Control signals
    bool reg_write = false;
    bool mem_to_reg = false;
    
    // Data
    uint64_t alu_result = 0;
    uint64_t mem_read_data = 0;
    uint64_t pc = 0;  // PC of instruction (for calculating return address)
    uint8_t rd = 0;
    uint32_t instruction = 0;
    bool valid = false;
    
    void Clear() {
        reg_write = mem_to_reg = false;
        alu_result = mem_read_data = 0;
        pc = 0;
        rd = 0;
        instruction = 0;
        valid = false;
    }
};

#endif // PIPELINE_REGISTERS_H

