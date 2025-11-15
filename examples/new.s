lui x3, 0x10000
addi x4, x0, 42
sw x4, 0(x3) # write to data region (0x10000000)
lw x5, 0(x3) # first read (miss)
lw x6, 0(x3) # hit
lw x7, 0(x3) # hit
nop