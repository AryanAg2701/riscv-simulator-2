/**
 * @file vm_base.cpp
 * @brief File containing the base class for the virtual machine
 * @author Vishank
 */

// --- Minimal filesystem fix for macOS ---
 #include <filesystem>
  namespace fs = std::filesystem;
// --- End fix ---


#include "vm/vm_base.h"
#include "globals.h"
#include "config.h"

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <thread>

using namespace std;


void VmBase::LoadProgram(const AssembledProgram &program) {
    program_ = program;
    unsigned int counter = 0;

    for (const auto &instruction : program.text_buffer) {
        memory_controller_.WriteWord(counter, instruction);
        counter += 4;
    }

    program_size_ = counter;
    AddBreakpoint(program_size_, false);

    unsigned int data_counter = 0;
    uint64_t base_data_address = vm_config::config.getDataSectionStart();

    auto align = [&](unsigned int alignment) {
        if (data_counter % alignment != 0)
            data_counter += alignment - (data_counter % alignment);
    };

    for (const auto &data : program.data_buffer) {
        switch (data.type) {
            case AssembledProgram::DataType::U8:
                align(1);
                memory_controller_.WriteByte(base_data_address + data_counter, data.value.u8);
                data_counter += 1;
                break;

            case AssembledProgram::DataType::U16:
                align(2);
                memory_controller_.WriteHalfWord(base_data_address + data_counter, data.value.u16);
                data_counter += 2;
                break;

            case AssembledProgram::DataType::U32:
                align(4);
                memory_controller_.WriteWord(base_data_address + data_counter, data.value.u32);
                data_counter += 4;
                break;

            case AssembledProgram::DataType::U64:
                align(8);
                memory_controller_.WriteDoubleWord(base_data_address + data_counter, data.value.u64);
                data_counter += 8;
                break;

            case AssembledProgram::DataType::F32: {
                align(4);
                uint32_t float_as_int;
                std::memcpy(&float_as_int, &data.value.f32, sizeof(float));
                memory_controller_.WriteWord(base_data_address + data_counter, float_as_int);
                data_counter += 4;
                break;
            }

            case AssembledProgram::DataType::F64: {
                align(8);
                uint64_t double_as_int;
                std::memcpy(&double_as_int, &data.value.f64, sizeof(double));
                memory_controller_.WriteDoubleWord(base_data_address + data_counter, double_as_int);
                data_counter += 8;
                break;
            }

            case AssembledProgram::DataType::STR:
                align(1);
                for (size_t i = 0; i < data.s.size(); i++) {
                    memory_controller_.WriteByte(base_data_address + data_counter, static_cast<uint8_t>(data.s[i]));
                    data_counter += 1;
                }
                break;
        }
    }

    std::cout << "VM_PROGRAM_LOADED" << std::endl;
    output_status_ = "VM_PROGRAM_LOADED";
if (!program_.instruction_number_line_number_mapping.empty() &&
    !program_.instruction_number_disassembly_mapping.empty()) {
    DumpState(globals::vm_state_dump_file_path);
} else {
    std::cerr << "DumpState skipped: instruction/disassembly mappings empty or missing\n";
}}

uint64_t VmBase::GetProgramCounter() const {
    return program_counter_;
}

void VmBase::UpdateProgramCounter(int64_t value) {
    program_counter_ = static_cast<uint64_t>(static_cast<int64_t>(program_counter_) + value);
}

auto sign_extend = [](uint32_t value, unsigned int bits) -> int32_t {
    int32_t mask = 1 << (bits - 1);
    return (value ^ mask) - mask;
};

int32_t VmBase::ImmGenerator(uint32_t instruction) {
    int32_t imm = 0;
    uint8_t opcode = instruction & 0b1111111;

    switch (opcode) {
        case 0b0010011:
        case 0b0000011:
        case 0b1100111:
        case 0b0001111:
        case 0b0000111:
            imm = (instruction >> 20) & 0xFFF;
            imm = sign_extend(imm, 12);
            break;

        case 0b0100011:
        case 0b0100111:
            imm = ((instruction >> 7) & 0x1F) | ((instruction >> 25) & 0x7F) << 5;
            imm = sign_extend(imm, 12);
            break;

        case 0b1100011:
            imm = ((instruction >> 8) & 0xF)
                | ((instruction >> 25) & 0x3F) << 4
                | ((instruction >> 7) & 0x1) << 10
                | ((instruction >> 31) & 0x1) << 11;
            imm <<= 1;
            imm = sign_extend(imm, 13);
            break;

        case 0b0110111:
        case 0b0010111:
            imm = (instruction & 0xFFFFF000) >> 12;
            break;

        case 0b1101111:
            imm = ((instruction >> 21) & 0x3FF)
                | ((instruction >> 20) & 0x1) << 10
                | ((instruction >> 12) & 0xFF) << 11
                | ((instruction >> 31) & 0x1) << 19;
            imm <<= 1;
            imm = sign_extend(imm, 21);
            break;

        case 0b0110011:
        case 0b1010011:
            imm = 0;
            break;

        default:
            imm = 0;
            break;
    }

    return imm;
}

void VmBase::AddBreakpoint(uint64_t val, bool is_line) {
    if (is_line) {
        if (program_.line_number_instruction_number_mapping.find(val) == program_.line_number_instruction_number_mapping.end()) {
            std::cerr << "Invalid line number: " << val << std::endl;
            return;
        }
        uint64_t line = val;
        uint64_t bp = program_.line_number_instruction_number_mapping[line] * 4;
        if (CheckBreakpoint(bp)) {
            std::cerr << "Breakpoint already exists at line: " << line << std::endl;
            return;
        }
        breakpoints_.emplace_back(bp);
    } else {
        if (val % 4 != 0) {
            std::cerr << "Invalid instruction address: " << val << ". Must be a multiple of 4." << std::endl;
            return;
        }
        if (CheckBreakpoint(val)) {
            std::cerr << "Breakpoint already exists at address: " << val << std::endl;
            return;
        }
        breakpoints_.emplace_back(val);
    }

    DumpState(globals::vm_state_dump_file_path);
}

void VmBase::RemoveBreakpoint(uint64_t val, bool is_line) {
    if (is_line) {
        if (program_.line_number_instruction_number_mapping.find(val) == program_.line_number_instruction_number_mapping.end()) {
            std::cerr << "Invalid line number: " << val << std::endl;
            return;
        }
        uint64_t line = val;
        uint64_t bp = program_.line_number_instruction_number_mapping[line] * 4;
        if (!CheckBreakpoint(bp)) {
            std::cerr << "No breakpoint exists at line: " << line << std::endl;
            return;
        }
        breakpoints_.erase(std::remove(breakpoints_.begin(), breakpoints_.end(), bp), breakpoints_.end());
    } else {
        if (val % 4 != 0) {
            std::cerr << "Invalid instruction address: " << val << ". Must be a multiple of 4." << std::endl;
            return;
        }
        if (!CheckBreakpoint(val)) {
            std::cerr << "No breakpoint exists at address: " << val << std::endl;
            return;
        }
        breakpoints_.erase(std::remove(breakpoints_.begin(), breakpoints_.end(), val), breakpoints_.end());
    }

    DumpState(globals::vm_state_dump_file_path);
}

bool VmBase::CheckBreakpoint(uint64_t address) {
    return std::find(breakpoints_.begin(), breakpoints_.end(), address) != breakpoints_.end();
}

void VmBase::PrintString(uint64_t address) {
    while (true) {
        char c = memory_controller_.ReadByte(address);
        if (c == '\0') break;
        std::cout << c;
        address++;
    }
}

void VmBase::DumpState(const std::filesystem::path &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for dumping VM state: " << filename << std::endl;
        return;
    }

    try {
        file << "{\n";
        unsigned int instruction_number = program_counter_ / 4;
        file << "    \"program_counter\": \"0x"
             << std::hex << std::setw(8) << std::setfill('0') << program_counter_
             << std::dec << std::setfill(' ') << "\",\n";

        unsigned int current_line = 0;
        auto it_line = program_.instruction_number_line_number_mapping.find(instruction_number);
        if (it_line != program_.instruction_number_line_number_mapping.end())
            current_line = it_line->second;

        file << "    \"current_line\": " << current_line << ",\n";
        file << "    \"current_instruction\": \"0x"
             << std::hex << std::setw(8) << std::setfill('0') << current_instruction_
             << std::dec << std::setfill(' ') << "\",\n";

        unsigned int disasm_line = 0;
        auto it_dis = program_.instruction_number_disassembly_mapping.find(instruction_number);
        if (it_dis != program_.instruction_number_disassembly_mapping.end())
            disasm_line = it_dis->second;

        file << "    \"disassembly_line_number\": " << disasm_line << ",\n";
        file << "    \"cycle_count\": " << cycle_s_ << ",\n";
        file << "    \"instructions_retired\": " << instructions_retired_ << ",\n";
        file << "    \"cpi\": " << cpi_ << ",\n";
        file << "    \"ipc\": " << ipc_ << ",\n";
        file << "    \"stall_cycles\": " << stall_cycles_ << ",\n";
        file << "    \"branch_mispredictions\": " << branch_mispredictions_ << ",\n";

        file << "    \"breakpoints\": [";
        for (size_t i = 0; i < breakpoints_.size(); ++i) {
            unsigned int bp_instr = breakpoints_[i] / 4;
            auto it_bp = program_.instruction_number_line_number_mapping.find(bp_instr);
            file << (it_bp != program_.instruction_number_line_number_mapping.end() ? it_bp->second : 0);
            if (i < breakpoints_.size() - 1) file << ", ";
        }
        file << "],\n";
        file << "    \"output_status\": \"" << output_status_ << "\"\n";
        file << "}\n";
        file.close();
    } catch (const std::exception &e) {
        std::cerr << "Exception during DumpState: " << e.what() << std::endl;
        if (file.is_open()) file.close();
    } catch (...) {
        std::cerr << "Unknown exception during DumpState" << std::endl;
        if (file.is_open()) file.close();
    }
}

void VmBase::ModifyRegister(const std::string &reg_name, uint64_t value) {
    registers_.ModifyRegister(reg_name, value);
}
