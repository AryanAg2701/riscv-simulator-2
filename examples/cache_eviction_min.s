lui x3, 0x10000
addi x4, x0, 0x100   # STRIDE = 256 bytes

lw  x5, 0(x3)        # fill 1
add x6, x3, x4
lw  x7, 0(x6)        # fill 2
add x6, x6, x4
lw  x8, 0(x6)        # fill 3
add x6, x6, x4
lw  x9, 0(x6)        # fill 4
add x6, x6, x4
lw  x10, 0(x6)       # fill 5 -> eviction if associativity=4

lw  x11, 0(x3)       # reaccess first (likely miss if evicted)
nop


