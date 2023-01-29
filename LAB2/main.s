.cpu cortex-m0
.thumb
.syntax unified
.fpu softvfp

.data
.align 4
// Your global variables go here
.global str, source, sum, i, x, a, z, tmp
str: .string "hello, 01234 world! 56789=-"
source: .word 3, 4, 7, 11, 18, 29, 47, 76, 123, 199, 322
sum: .word 0
i: .word 0
x: .byte 0

a: .byte 0x61
z: .byte 0x7a
tmp: .string "\0"
.text
.global intsub
intsub:
    // Your code for intsub goes here
for1: //for(int i = 0
	ldr r0, =i
	ldr r1, [r0]
	movs r1, #0
	str r1, [r0]
check1: //for(int i = 0; i < 10
	ldr r0, =i
	ldr r1, [r0]
	cmp r1, #10
	bge endfor1
body1:
if1:

	ldr r0, =sum
	ldr r1, [r0]
	cmp r1, #25
	bge else1			//if (sum < 25)

	ldr r0, =i
	ldr r1, [r0]
	lsls r0, r1, #2 	// r0 = the offset [i]
	ldr r1, =source
	ldr r2, [r1, r0] 	// r2 = source[i]
	adds r0, #4			// r0 = net offset [i+1]
	ldr r3, [r1, r0] 	// r3 = source[i+1]
	adds r3, r2			// r3 = source[i] + source[i+1]
	ldr r0, =sum
	ldr r1, [r0]		// loads r1 with sum
	adds r1, r3 		// r3 = sum += source[i] + source[i+1]
	str r1, [r0]		// sum += source[i] + source[i+1]
	b update1
else1:
	ldr r0, =i
	ldr r1, [r0]
	lsls r0, r1, #2 	// r0 = the offset [i]
	ldr r1, =source
	ldr r2, [r1, r0] 	// r2 = source[i]
	movs r0, #2
	muls r2, r0			// r2 = 2 * source[i]
	ldr r0, =sum
	ldr r1, [r0]
	subs r1, r2			// r2 = sum -= 2 * source[i]
	str r1, [r0]		// sum -= 2 * source[i]
	b update1



update1:					// for(int i = 0; i < 10; i++)
	ldr r0, =i
	ldr r1, [r0]
	adds r1, #1
	strb r1, [r0]
	b check1


endfor1:

    bx lr



.global charsub
charsub:
    // Your code for charsub goes here
for2:
//	ldr r0, =x
//	ldr r1, [r0]
//	movs r1, #0
//	str r1, [r0]
check2:
	ldr r0, =x
	ldrb r1, [r0]
	lsls r0, r1, #0 	// r0 = the offset [x]
	ldr r1, =str
	ldrb r2, [r1, r0] 	// r2 = str[x]
	adds r0, #1			// r0 = net offset [i+1]
	ldrb r3, [r1, r0] 	// r3 = str[x+1]

	cmp r2, #0
	beq endfor2
	cmp r3, #0
	beq endfor2


body2:
if2:
	ldr r0, =x
	ldrb r1, [r0]
	lsls r0, r1, #0 	// r0 = the offset [x]
	ldr r1, =str
	ldrb r3, [r1, r0] 	// r3 = str[x]
	ldr r0, =a
	ldrb r1, [r0]
	cmp r3, r1			// str[x] >= 'a'
	ble next2
	ldr r0, =z
	ldrb r1, [r0]
	cmp r3, r1
	bgt next2			// str[x] <= 'z'

then2:
	ldr r0, =tmp
	ldrb r3, [r0]		// r3 = temp
	ldr r0, =x
	ldrb r1, [r0]
	lsls r0, r1, #0 	// r0 = the offset [x]
	ldr r1, =str
	ldrb r2, [r1, r0] 	// r2 = str[x]

	movs r3, r2	        // char tmp = str[x]
						// r3 = temp

	adds r0, #1			// r0 = net offset [x+1]
	ldrb r2, [r1, r0] 	// r2 = str[x+1]
	subs r0, #1
	strb r2, [r1, r0]	// str[x] = str[x+1]
	adds r0, #1
	strb r3, [r1, r0]	// str[x+1] = tmp



next2:
	ldr r0, =x
	ldrb r1, [r0]
	adds r1, #2
	strb r1, [r0]
	b check2

endfor2:

    bx lr


.global login
login: .string "ssvanda" // Make sure you put your login here.
.align 2
.global main
main:
    bl autotest // uncomment AFTER you debug your subroutines
    bl intsub
    bl charsub
    bkpt
