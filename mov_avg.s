/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1's Name here: Venkatesh Ksheerabthi Nathan (A0308961X)
@ Write Student 2's Name here: WXYZ (A0000007X)
@ Register usage:
@ R0 = N (input: loop counter, counts down to 0), later reused for return value
@ R1 = accel_buff (input: pointer to int array)
@ R3 = accumulator (sum)
@ R4 = temporary value loaded from buffer
@ write your program from here:
mov_avg:
 PUSH {r2-r11, lr}

 MOV r3, #0          @ sum = 0

loop:
 LDR r4, [r1], #4    @ r4 = *r1, then r1 += 4 (load and advance pointer)
 ADD r3, r3, r4      @ sum += value
 SUBS r0, r0, #1     @ N-- and set flags
 BNE loop            @ if N != 0, keep looping

 ASR r0, r3, #2      @ result = sum / 4 (arithmetic shift right by 2)

 POP {r2-r11, pc}
