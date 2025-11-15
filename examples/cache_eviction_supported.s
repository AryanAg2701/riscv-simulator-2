    .data
buf: .space 4096

    .text
_start:
    la   x5, buf
    li   x6, 256

    lw   x7, 0(x5)
    add  x20, x5, x6
    lw   x8, 0(x20)
    add  x21, x20, x6
    lw   x9, 0(x21)
    add  x22, x21, x6
    lw   x10, 0(x22)
    add  x23, x22, x6
    lw   x11, 0(x23)

    lw   x12, 0(x5)
    nop


