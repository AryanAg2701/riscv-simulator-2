# No pseudos. Heavy memory + hazards + branches to differentiate modes and stress D-cache.

    # Base = 0x10000000 (data region)
    lui   x3, 0x10000

    # N = 1024 (elements)
    addi  x9, x0, 1024

    # i = 0
    addi  x4, x0, 0

# ------------------- Initialize array: A[i] = 2*i + 1 -------------------
init_loop:
    slli  x7, x4, 2          # addr = base + i*4
    add   x7, x7, x3
    add   x6, x4, x4         # val = i*2
    addi  x6, x6, 1          # val = i*2 + 1
    sw    x6, 0(x7)
    addi  x4, x4, 1
    slt   x14, x4, x9
    bne   x14, x0, init_loop

# ------------------- Sequential pass with load-use hazards ----------------
    addi  x10, x0, 0         # sum = 0
    addi  x4,  x0, 0         # i = 0

seq_loop:
    slli  x7, x4, 2
    add   x7, x7, x3
    lw    x11, 0(x7)         # load
    add   x10, x10, x11      # immediate use (hazard)
    andi  x12, x11, 1        # branch on parity (unpredictable-ish)
    beq   x12, x0, seq_even
    addi  x13, x0, 1
    jal   x0, seq_after
seq_even:
    addi  x13, x0, 2
seq_after:
    addi  x4, x4, 1
    slt   x14, x4, x9
    bne   x14, x0, seq_loop

# ------------------- Stride-conflict pass (set thrashing) -----------------
    # stride = 4096 (64 sets * 64B line), maps different lines to same index
    lui   x16, 0x1           # x16 = 4096
    add   x20, x3, x0        # a0 = base
    add   x21, x20, x16      # a1 = base + 4096
    add   x22, x21, x16      # a2 = base + 8192
    add   x23, x22, x16      # a3 = base + 12288
    add   x24, x23, x16      # a4 = base + 16384

    addi  x4,  x0, 0         # t = 0
    addi  x17, x0, 64        # outer iters
    addi  x11, x0, 0         # branch toggle accumulator

outer_loop:
    # Access 5 conflicting lines to force eviction in 4-way cache
    lw    x5,  0(x20)
    add   x10, x10, x5
    lw    x6,  0(x21)
    add   x10, x10, x6
    lw    x7,  0(x22)
    add   x10, x10, x7
    lw    x8,  0(x23)
    add   x10, x10, x8
    lw    x9,  0(x24)        # 5th line → eviction
    add   x10, x10, x9

    # Unpredictable branch on sum's LSB to stress predictor
    andi  x12, x10, 1
    beq   x12, x0, outer_skip
    addi  x11, x11, 1
    jal   x0, outer_cont
outer_skip:
    addi  x11, x11, -1
outer_cont:
    addi  x4,  x4,  1
    slt   x14, x4,  x17
    bne   x14, x0, outer_loop

    nop


