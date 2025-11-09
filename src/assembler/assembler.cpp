#include "assembler/assembler.h"
#include "assembler/assembled_program.h"
#include "assembler/parser.h"
#include "utils.h"
#include "globals.h"

#include <string>
#include <memory>
#include <stdexcept>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>

AssembledProgram assemble(const std::string &filename) {
    std::unique_ptr<Lexer> lexer;
    try {
        lexer = std::make_unique<Lexer>(filename);
    } catch (const std::runtime_error &e) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::vector<Token> tokens = lexer->getTokenList();

    Parser parser(lexer->getFilename(), tokens);
    parser.parse();

    AssembledProgram program;
    program.filename = filename;

    if (parser.getErrorCount() == 0) {
        std::vector<uint32_t> machine_code_bits = generateMachineCode(parser.getIntermediateCode());

        auto &pbuf = parser.getDataBuffer();
        program.data_buffer.reserve(pbuf.size());

        for (const auto &p : pbuf) {
            AssembledProgram::DataEntry entry;
            switch (p.type) {
                case ParserDataEntry::DataType::U8:
                    entry.type = AssembledProgram::DataType::U8;
                    entry.value.u8 = static_cast<uint8_t>(p.value.u8);
                    break;
                case ParserDataEntry::DataType::U16:
                    entry.type = AssembledProgram::DataType::U16;
                    entry.value.u16 = static_cast<uint16_t>(p.value.u16);
                    break;
                case ParserDataEntry::DataType::U32:
                    entry.type = AssembledProgram::DataType::U32;
                    entry.value.u32 = static_cast<uint32_t>(p.value.u32);
                    break;
                case ParserDataEntry::DataType::U64:
                    entry.type = AssembledProgram::DataType::U64;
                    entry.value.u64 = static_cast<uint64_t>(p.value.u64);
                    break;
                case ParserDataEntry::DataType::F32:
                    entry.type = AssembledProgram::DataType::F32;
                    entry.value.f32 = p.value.f32;
                    break;
                case ParserDataEntry::DataType::F64:
                    entry.type = AssembledProgram::DataType::F64;
                    entry.value.f64 = p.value.f64;
                    break;
                case ParserDataEntry::DataType::STR:
                    entry.type = AssembledProgram::DataType::STR;
                    entry.s = p.s;
                    break;
                default:
                    break;
            }
            program.data_buffer.emplace_back(std::move(entry));
        }


        program.intermediate_code = parser.getIntermediateCode();
        program.text_buffer = std::move(machine_code_bits);
        program.instruction_number_line_number_mapping = parser.getInstructionNumberLineNumberMapping();

        program.line_number_instruction_number_mapping = [&]() {
            std::map<unsigned int, unsigned int> mapping;
            if (program.instruction_number_line_number_mapping.empty()) return mapping;
            unsigned int prev_instr = 0;
            unsigned int prev_line = 1;
            for (const auto &pair : program.instruction_number_line_number_mapping) {
                unsigned int line = pair.second;
                for (unsigned int i = prev_line; i <= line; ++i) {
                    mapping[i] = prev_instr;
                }
                prev_instr += 1;
                prev_line = line + 1;
            }
            return mapping;
        }();

        program.symbol_table = parser.getSymbolTable();

        DumpDisasssembly(globals::disassembly_file_path, program);
        DumpNoErrors(globals::errors_dump_file_path);

    } else {
        DumpErrors(globals::errors_dump_file_path, parser.getErrors());
        if (globals::verbose_errors_print) parser.printErrors();
        throw std::runtime_error("Failed to parse file: " + filename);
    }

    return program;
}
