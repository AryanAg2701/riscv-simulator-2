#ifndef PARSER_H
#define PARSER_H

#include "assembler/tokens.h"
#include "assembler/code_generator.h"
#include "assembler/errors.h"

#include <map>
#include <string>
#include <vector>
#include <variant>
#include <cstdint>
#include <utility>

struct ParseError {
    unsigned int line;
    std::string message;
    ParseError(unsigned int line, std::string message) : line(line), message(std::move(message)) {}
};

struct ErrorTracker {
    unsigned int count = 0;
    std::vector<ParseError> parse_errors;
    std::vector<std::variant<
        errors::SyntaxError,
        errors::UnexpectedTokenError,
        errors::ImmediateOutOfRangeError,
        errors::MisalignedImmediateError,
        errors::UnexpectedOperandError,
        errors::InvalidLabelRefError,
        errors::LabelRedefinitionError,
        errors::InvalidRegisterError
    >> all_errors;
};

// Modern variant-based representation for .data segment
struct ParserDataEntry {
    enum class DataType {
        U8, U16, U32, U64, F32, F64, STR
    } type;
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


struct SymbolData {
    uint64_t address;
    uint64_t line_number;
    bool isData;
};

class Parser {
private:
    std::string filename_;
    std::vector<Token> tokens_;
    size_t pos_ = 0;
    unsigned int instruction_index_ = 0;
    ErrorTracker errors_;

    std::vector<ParserDataEntry> data_buffer_;
    uint64_t data_index_ = 0;

    std::map<std::string, SymbolData> symbol_table_;
    std::vector<unsigned int> back_patch_;
    std::vector<std::pair<ICUnit, bool>> intermediate_code_;
    std::map<unsigned int, unsigned int> instruction_number_line_number_mapping_;

    Token prevToken();
    Token currentToken();
    Token nextToken();
    Token peekToken(int n);
    void skipCurrentLine();
    void recordError(const ParseError &error);

    bool parse_O_GPR_C_GPR_C_GPR();
    bool parse_O_GPR_C_GPR_C_I();
    bool parse_O_GPR_C_I();
    bool parse_O_GPR_C_GPR_C_IL();
    bool parse_O_GPR_C_GPR_C_DL();
    bool parse_O_GPR_C_IL();
    bool parse_O_GPR_C_DL();
    bool parse_O_GPR_C_I_LP_GPR_RP();
    bool parse_O();
    bool parse_pseudo();
    bool parse_O_GPR_C_CSR_C_GPR();
    bool parse_O_GPR_C_CSR_C_I();
    bool parse_O_FPR_C_FPR_C_FPR_C_FPR();
    bool parse_O_FPR_C_FPR_C_FPR_C_FPR_C_RM();
    bool parse_O_FPR_C_FPR_C_FPR();
    bool parse_O_FPR_C_FPR_C_FPR_C_RM();
    bool parse_O_FPR_C_FPR();
    bool parse_O_FPR_C_FPR_C_RM();
    bool parse_O_FPR_C_GPR();
    bool parse_O_FPR_C_GPR_C_RM();
    bool parse_O_GPR_C_FPR();
    bool parse_O_GPR_C_FPR_C_RM();
    bool parse_O_GPR_C_FPR_C_FPR();
    bool parse_O_FPR_C_I_LP_GPR_RP();

    void parseDataDirective();
    void parseTextDirective();
    void parseBSSDirective();

public:
    explicit Parser(std::string filename, const std::vector<Token> &tokens)
        : filename_(std::move(filename)), tokens_(tokens) {}

    void parse();
    unsigned int getErrorCount() const { return errors_.count; }
    const std::vector<ParseError> &getErrors() const { return errors_.parse_errors; }

    std::vector<ParserDataEntry> &getDataBuffer() { return data_buffer_; }
    const std::vector<std::pair<ICUnit, bool>> &getIntermediateCode() const { return intermediate_code_; }
    const std::map<unsigned int, unsigned int> &getInstructionNumberLineNumberMapping() const { return instruction_number_line_number_mapping_; }
    const std::map<std::string, SymbolData> &getSymbolTable() const { return symbol_table_; }

    void printErrors() const;
    void printSymbolTable() const;
    void printDataBuffers() const;
    void printIntermediateCode() const;
};

#endif // PARSER_H
