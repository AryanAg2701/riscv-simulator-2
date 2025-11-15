    .data
buf: .space 4096

    .text
    .globl _start
_start:
    # Assumes typical config where multiple addresses map to same set.
    la    x5, buf
    li    x6, 256         # STRIDE (bytes)

    # Access associativity+1 lines to force an eviction in that set
    lw    x7, 0(x5)       # fill 1
    add   x20, x5, x6
    lw    x8, 0(x20)      # fill 2
    add   x21, x20, x6
    lw    x9, 0(x21)      # fill 3
    add   x22, x21, x6
    lw    x10, 0(x22)     # fill 4
    add   x23, x22, x6
    lw    x11, 0(x23)     # fill 5 -> should evict one (if associativity=4)

    # Re-access first address; expect a miss if it was evicted
    lw    x12, 0(x5)

    nop


