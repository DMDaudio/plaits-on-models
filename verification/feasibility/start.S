| Bare-metal entry for the Plaits bench under Unicorn.
	.text
	.globl _start
_start:
	move.l	#__stack_top, %sp
	| zero .bss
	lea	__bss_start, %a0
	lea	__bss_end, %a1
1:	cmp.l	%a1, %a0
	bcc.s	2f
	clr.l	(%a0)+
	bra.s	1b
2:	| run C++ constructors
	lea	__init_array_start, %a2
3:	cmp.l	#__init_array_end, %a2
	bcc.s	4f
	move.l	(%a2)+, %a0
	jsr	(%a0)
	bra.s	3b
4:	jsr	bench_main
	.globl bench_done
bench_done:
	nop
	bra.s	bench_done
