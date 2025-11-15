    .data
arr: .space 64

    .text
    .globl _start
_start:
    # Initialize base pointer to arr
    la    x5, arr

    # First load (expected miss, fills line)
    lw    x6, 0(x5)

    # Repeat loads (expected hits)
    lw    x7, 0(x5)
    lw    x8, 0(x5)
    lw    x9, 0(x5)
    lw    x10, 0(x5)
    lw    x11, 0(x5)
    lw    x12, 0(x5)
    lw    x13, 0(x5)

    # Touch nearby words within the same block to show block utilization
    lw    x14, 4(x5)
    lw    x15, 8(x5)
    lw    x16, 12(x5)

    nop


