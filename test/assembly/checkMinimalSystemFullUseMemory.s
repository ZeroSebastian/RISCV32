.global main
main:

.equ CONSTANT, 		0x100000
.equ PERIPH_OUT,   	0x00008000

main:

	# reserve space for data
	li sp, 0
	li a0, 0x2000
	add sp, sp, a0

	# clear register
	li a0, 0

	# load peripheral address
	li a2, PERIPH_OUT

	# load compare value
	li a1, CONSTANT
	
	# load address of counter
	la t0, KAUNTER
	
loop:
	# increment
	addi a0, a0, 1

	# check loopcounter
	bge a0, a1, blink

	j loop

blink:
	# reset loop counter
	li a0, 0
	
	# load a3
	lw a5, KAUNTER
	addi a5, a5, 1
	# store
	sw a5, 0(t0)
	
	# write to peripheral
	sb a5, 0(a2)

	j loop

KAUNTER: .int 0x00000000
