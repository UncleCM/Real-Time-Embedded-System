.global _func_asm

_func_asm:
	mov r0, #225
	mov r1, #64
	add r2, r0, r1
	str r2, [r1, #0]
	ldr r3, [r1]
	b _accumulate
	
_accumulate:
	add r4, r4, #1
	