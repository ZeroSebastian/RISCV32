.section .text
.global _start

_start:

	#load 0 sign extension
	lb x2, 1(x0)
	lh x3, 1(x0)
	lw x4, 1(x0)
	j test

	#store some values
	lw x1, (x0)
	sb x1, 1(x0)
	sh x1, 2(x0)
	sw x1, 4(x0)

test:
	#load 1 sign extension
	lb x2, 1(x0)
loop:
	lh x3, 1(x0)
	lw x4, 1(x0)
	lbu x5, 1(x0)
	lhu x6, 1(x0)

	j loop
