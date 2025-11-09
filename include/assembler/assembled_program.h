#ifndef ASSEMBLED_PROGRAM_H
#define ASSEMBLED_PROGRAM_H

#include <string>
#include <vector>
#include <map>
#include <cstdint>

struct ICUnit;      // forward declaration (defined elsewhere)
struct SymbolData;  // forward declaration (defined elsewhere)

struct AssembledProgram {
    enum class DataType { U8, U16, U32, U64, F32, F64, STR };

    struct DataEntry {
        DataType type;
        std::string s; // used only for STR
        union Value {
            uint8_t u8;
            uint16_t u16;
            uint32_t u32;
            uint64_t u64;
            float f32;
            double f64;
            Value() : u64(0) {}
        } value;
    };

    std::string filename;
    std::vector<DataEntry> data_buffer;
    std::vector<uint32_t> text_buffer;
    std::vector<std::pair<ICUnit, bool>> intermediate_code;
    std::map<unsigned int, unsigned int> instruction_number_disassembly_mapping;
    std::map<unsigned int, unsigned int> instruction_number_line_number_mapping;
    std::map<unsigned int, unsigned int> line_number_instruction_number_mapping;
    std::map<std::string, SymbolData> symbol_table;
};

#endif // ASSEMBLED_PROGRAM_H
