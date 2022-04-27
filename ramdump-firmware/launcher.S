.syntax unified
.cpu cortex-m0
.fpu softvfp
.thumb

.set SYS_MODE_REG, 0x40000080
.set REMAP_BIT   , 0x40000000

.global vtable
.global main

.type vtable, %object
vtable:
    .word _estack
    .word starter
.size vtable, .-vtable

starter:
        LDR       R1, = SYS_MODE_REG
        LDR       R0, [R1]
        LDR       R2, = REMAP_BIT
        ORRS      R0, R0, R2
        STR       R0, [R1]

        LDR     R0, = _estack
        MSR     MSP, R0

	bl main
inf:
	b inf
	nop
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
	bl starter
