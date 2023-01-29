.cpu cortex-m0
.thumb
.syntax unified
.fpu softvfp

//==================================================================
// ECE 362 Lab Experiment 5
// Timers
//==================================================================

// RCC configuration registers
.equ  RCC,      0x40021000
.equ  AHBENR,   0x14
.equ  GPIOCEN,  0x00080000
.equ  GPIOBEN,  0x00040000
.equ  GPIOAEN,  0x00020000
.equ  APB1ENR,  0x1c
.equ  TIM6EN,   1<<4
.equ  TIM7EN,   1<<5

// NVIC configuration registers
.equ NVIC, 0xe000e000
.equ ISER, 0x100
.equ ICER, 0x180
.equ ISPR, 0x200
.equ ICPR, 0x280
.equ IPR,  0x400
.equ TIM6_DAC_IRQn, 17
.equ TIM7_IRQn,     18

// Timer configuration registers
.equ TIM6, 0x40001000
.equ TIM7, 0x40001400
.equ TIM_CR1,  0x00
.equ TIM_CR2,  0x04
.equ TIM_DIER, 0x0c
.equ TIM_SR,   0x10
.equ TIM_EGR,  0x14
.equ TIM_CNT,  0x24
.equ TIM_PSC,  0x28
.equ TIM_ARR,  0x2c

// Timer configuration register bits
.equ TIM_CR1_CEN,  1<<0
.equ TIM_DIER_UDE, 1<<8
.equ TIM_DIER_UIE, 1<<0
.equ TIM_SR_UIF,   1<<0

// GPIO configuration registers
.equ  GPIOC,    0x48000800
.equ  GPIOB,    0x48000400
.equ  GPIOA,    0x48000000
.equ  MODER,    0x00
.equ  PUPDR,    0x0c
.equ  IDR,      0x10
.equ  ODR,      0x14
.equ  BSRR,     0x18
.equ  BRR,      0x28

//==========================================================================
// enable_ports  (Autotest 1)
// Enable RCC clock for GPIO ports B and C.
// Parameters: none
// Return value: none
.global enable_ports
enable_ports:
	// Student code goes below
	push {r4, lr}
	//enable the clock
	ldr r0, =RCC
	ldr r1, [r0, #AHBENR]
	//ldr r2, =GPIOAEN
	ldr r3, =GPIOBEN
	ldr r4, =GPIOCEN
	//orrs r1, r2
	orrs r1, r3
	orrs r1, r4
	str r1, [r0, #AHBENR]

    // Student code goes here
    // configure PB0-3 to be outputs
	ldr r0, =GPIOB
	ldr r1, [r0, #MODER]
	ldr r2, =0xff
	bics r1, r2
	ldr r2, =0x55
	orrs r1, r2
	str r1, [r0, #MODER]
	 // Student code goes here\
	 // Configures pins PB4 – PB7 to be inputs
	ldr r0, =GPIOB
	ldr r1, [r0, #MODER]
	ldr r2, =0xff00
	bics r1, r2
	str r1, [r0, #MODER]
	// Configures pins PB4 – PB7 to be internally pulled low
	ldr r1, [r0, #PUPDR]		// enables pull down resistor for PB2
	ldr r2, =0xff00
	bics r1, r2
	ldr r2, =0xaa00
	orrs r1, r2
	str r1, [r0, #PUPDR]
	// Configures pins PC0 – PC10 to be outputs
	ldr r0, =GPIOC
	ldr r1, [r0, #MODER]
	ldr r2, =0x3fffff
	bics r1, r2
	ldr r2, =0x155555
	orrs r1, r2
	str r1, [r0, #MODER]
	pop {r4, pc}
	// Student code goes above


//==========================================================================
// Timer 6 Interrupt Service Routine  (Autotest 2)
// Parameters: none
// Return value: none
// Write your entire subroutine below

.global TIM6_DAC_IRQHandler
.type TIM6_DAC_IRQHandler, %function
TIM6_DAC_IRQHandler:
	push {r4, lr}

	ldr r0,=TIM6
	ldr r1,[r0,#TIM_SR] // read status reg
	ldr r2,=TIM_SR_UIF
	bics r1,r2 // turn off UIF
	str r1,[r0,#TIM_SR] // write it

	ldr r4, =0x40
	ldr r3, =GPIOC
    ldr r2, [r3, #ODR]
	eors r2, r4
    str r2, [r3, #ODR]
	pop {r4, pc}


//==========================================================================
// setup_tim6  (Autotest 3)
// Configure timer 6
// Parameters: none
// Return value: none
.global setup_tim6
setup_tim6:
	push {lr}
	// Student code goes below
	ldr r0, =RCC
	ldr r1, [r0, #APB1ENR]
	ldr r2, =TIM6EN
	orrs r1, r2
	str r1, [r0, #APB1ENR]

	ldr r0,=TIM6
	ldr r1,=48000-1
	str r1,[r0,#TIM_PSC]
	ldr r1,=500-1
	str r1,[r0,#TIM_ARR]

	ldr r0,=TIM6
	ldr r1,[r0,#TIM_DIER]
	ldr r2,=TIM_DIER_UIE
	orrs r1,r2
	str r1,[r0,#TIM_DIER]

	ldr r0,=TIM6
	ldr r1,[r0,#TIM_CR1]
	ldr r2,=TIM_CR1_CEN
	orrs r1,r2
	str r1,[r0,#TIM_CR1]

	ldr r0,=NVIC
	ldr r1,=ISER
	ldr r2,=(1<<TIM6_DAC_IRQn)
	str r2,[r0,r1]
	// Student code goes above
	pop  {pc}

.data
.global display
display: .byte 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07
.global history
history: .space 16
.global offset
offset: .byte 0
.text

//==========================================================================
// show_digit  (Autotest 4)
// Setup Port C outputs to show the digit for the current
// value of the offset variable.
// Parameters: none
// Return value: none
// Write your entire subroutine below.
.global show_digit
show_digit:
	push {r4, r5, r6, lr}
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =display
	ldr r3, =offset
	ldrb r3, [r3]
	ldr r4, =0x7
	ands r3, r4				//int off = offset & 7;
//	movs r7, r3
//	ldr r3, =offset
	ldrb r5, [r2, r3]		//display[off]
	ldr r6, =0x8
	lsls r3, r6				//(off << 8)
	orrs r5, r3
	//orrs r5, r1

	str r5, [r0, #ODR]


	pop  {r4, r5,r6, pc}
//==========================================================================
// get_cols  (Autotest 5)
// Return the current value of the PC8 - PC4 pins.
// Parameters: none
// Return value: 4-bit result of columns active for the selected row
// Write your entire subroutine below.
.global get_cols
get_cols:
	push {r4, r5, r6, lr}
	ldr r0, =GPIOB
	ldr r1, [r0, #IDR]
	ldr r2, =0x4
	lsrs r1, r2
	ldr r4, =0xf

	ands r1, r4
	movs r0, r1


	pop  {r4, r5, r6, pc}
//==========================================================================
// update_hist  (Autotest 6)
// Update history byte entries for the current row.
// Parameters: r0: cols: 4-bit value read from matrix columns
// Return value: none
// Write your entier subroutine below.
.global update_hist
update_hist:
	push {r4, r5, r6, r7, lr}
	ldr r1, =offset
	ldr r1, [r1]	 // r1 = offset
	ldr r2, =0x3
	ands r1, r2		 // r1 = int row = offset & 3;
	movs r7, #0		 // r7 = int i=0
	movs r2, #4		 // r2 = 4
for_update_hist:
	cmp r7, #4
	bge end_update_hist
	movs r3, r1		 // r3 = *row
	muls r3, r2		 // r3 = 4*row
	adds r3, r7		 // r3 = [4*row+i]
	ldr r6, =history
//	ldr r6, [r6]
	ldrb r4, [r6, r3]// r4 = history[4*row+i]
	lsls r4, #1		 // r4 = (history[4*row+i]<<1)
	movs r5, r0		 // r5 = *cols
	lsrs r5, r7		 // r5 = (cols>>i)
	movs r6, #1
	ands r5, r6		 // r5 = ((cols>>i)&1)
	adds r4, r5		 // r4 = (history[4*row+i]<<1) + ((cols>>i)&1)
	ldr r6, =history
	strb r4, [r6, r3]

	adds r7, #1
	b for_update_hist
end_update_hist:
	nop
	pop  {r4, r5, r6, r7, pc}
//==========================================================================
// set_row  (Autotest 7)
// Set PB3 - PB0 to represent the row being scanned.
// Parameters: none
// Return value: none
// Write your entire subroutine below.
.global set_row
set_row:
	push {r4, r5, r6, r7, lr}
	ldr r0, =GPIOB
	ldr r1, =offset
	ldr r1, [r1]
	ldr r2, =0x3
	ands r2, r1		// r2 = int row = offset & 3;
	movs r3, #1
	lsls r3, r2		// r2 = 1<<row
	ldr r4, =0xf0000
	orrs r3, r4		//0xf0000 | (1<<row)
	str r3, [r0, #BSRR]

	pop  {r4, r5, r6, r7, pc}
//==========================================================================
// Timer 7 Interrupt Service Routine  (Autotest 8)
// Parameters: none
// Return value: none
// Write your entire subroutine below
.global TIM7_IRQHandler
.type TIM7_IRQHandler, %function
TIM7_IRQHandler:
	push {r4, r5, r6, r7, lr}

	ldr r0,=TIM7
	ldr r1,[r0,#TIM_SR] // read status reg
	ldr r2,=TIM_SR_UIF
	bics r1,r2 // turn off UIF
	str r1,[r0,#TIM_SR] // write it

	bl show_digit
	bl get_cols
	bl update_hist
	ldr r1, =offset
	ldr r1, [r1]
	adds r1, #1
	ldr r2, =0x7
	ands r1, r2
	ldr r3, =offset
	strb r1, [r3]
	bl set_row

	pop  {r4, r5, r6, r7, pc}
//==========================================================================
// setup_tim7  (Autotest 9)
// Configure Timer 7.
// Parameters: none
// Return value: none
.global setup_tim7
setup_tim7:
	push {lr}
	// Student code goes below
	ldr r0, =RCC
	ldr r1, [r0, #APB1ENR]
	ldr r2, =TIM6EN
	bics r1, r2
	ldr r2, =TIM7EN
	orrs r1, r2
	str r1, [r0, #APB1ENR]			//clears out register for RCC CLK and updates tim7 clk

	ldr r0,=TIM7
	ldr r1,=4800-1
	str r1,[r0,#TIM_PSC]
	ldr r1,=10-1
	str r1,[r0,#TIM_ARR]

	ldr r0,=TIM7
	ldr r1,[r0,#TIM_DIER]
	ldr r2,=TIM_DIER_UIE
	orrs r1,r2
	str r1,[r0,#TIM_DIER]

	ldr r0,=TIM7
	ldr r1,[r0,#TIM_CR1]
	ldr r2,=TIM_CR1_CEN
	orrs r1,r2
	str r1,[r0,#TIM_CR1]

	ldr r0,=NVIC
	ldr r1,=ISER
	ldr r2,=(1<<TIM7_IRQn)
	str r2,[r0,r1]
	// Student code goes above
	pop  {pc}


//==========================================================================
// get_keypress  (Autotest 10)
// Wait for and return the number (0-15) of the ID of a button pressed.
// Parameters: none
// Return value: button ID
.global get_keypress
get_keypress:
	push {r4, r5, r6,lr}
	// Student code goes below
	movs r3, #0				// r3 = int i=0
get_keypress_wait:
	wfi
	ldr r0, =offset
	ldr r0, [r0]			//r0 = offset
	movs r1, #3
	ands r0, r1				//offset & 3		// r3 = int i=0
	cmp r0, #0

	bne get_keypress_wait
	beq get_keypress_for		// asm volatile ("wfi" : :)
get_keypress_for:
	cmp r3, #16
	bge get_keypress
	ldr r1, =history
	ldrb r2, [r1, r3]		//r2 = history[i]
	cmp r2, #1				//(history[i] == 1)

	beq end_get_keypress
	adds r3, #1
	b get_keypress_for
end_get_keypress:
	movs r0, r3
	// Student code goes above
	pop  {r4, r5, r6,pc}


//==========================================================================
// handle_key  (Autotest 11)
// Shift all symbols in the display left and add a new digit
// in the rightmost digit.
// ALSO: Create your "font" array just above.
// Parameters: ID of new button to display
// Return value: none
.global font
font: .byte 0x06, 0x5b, 0x4f, 0x77, 0x66, 0x6d, 0x7d, 0x7c, 0x07, 0x7f, 0x67, 0x39, 0x49, 0x3f, 0x76, 0x5e
.align 2
.global handle_key
handle_key:
	push {r4, r5, r6, r7, lr}
	// Student code goes below
	ldr r1, =0xf
	ands r0, r1			//r0 = key = key & 0xf
	movs r7, #0			// r7 = int i=0
	ldr r2, =display	//r2 = display
handle_key_for:
	cmp r7, #7			// i < 7
	bge handle_key_end

	adds r7, #1			//r7 = i+1
	ldrb r3, [r2, r7]	//r3 = display[i+1]
	subs r7, #1			//r7 = i
	strb r3, [r2, r7]	//r3 = display[i] = display[i+1]
	adds r7, #1
	b handle_key_for
handle_key_end:
	ldr r4, =font
	ldrb r5, [r4, r0]	// r5 =  font[key]
	movs r6, #7
	strb r5, [r2, r6]
	// Student code goes above

	pop {r4, r5, r6, r7, pc}

.global login
login: .string "ssvanda"
.align 2

//==========================================================================
// main
// Already set up for you.
// It never returns.
.global main
main:
//	bl  check_wiring
	//bl  autotest
	bl  enable_ports
	bl  setup_tim6
	bl  setup_tim7

endless_loop:
	bl   get_keypress
	bl   handle_key
	b    endless_loop
