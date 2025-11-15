    .data
var:    .word 0

    .text
    .globl _start
_start:
    lw    x6, 0(x5)
    addi  x7, x6, 1
    sw    x7, 0(x5)
    beq   x6, x0, L1
    addi  x8, x0, 0
    jal    L2
L1:
    addi  x8, x0, 1
L2:
    lw    x9, 0(x5)
    nop
