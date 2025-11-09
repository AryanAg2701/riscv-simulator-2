#ifndef ASSEMBLER_H
#define ASSEMBLER_H

#include "assembler/lexer.h"
#include "assembler/parser.h"
#include "assembler/assembled_program.h"
#include <string>
#include "assembler/code_generator.h"

AssembledProgram assemble(const std::string &filename);

#endif // ASSEMBLER_H
