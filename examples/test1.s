add x31, x31, x31
addi x0, x0, -353
srli x5, x5, 31
ecall
label1:
lb x10, 23(x0)
sd x10, -1023(x19)

beq x10, x7, label1

lui x3, 0x10000
auipc x3, 0x10
jal x0, label1
jal x0, 12422