.cpu cortex-m0
.thumb
.syntax unified
.fpu softvfp

//===================================================================
// ECE 362 Midterm Lab Practical
//===================================================================

// RCC configuration registers
.equ  RCC,      0x40021000
.equ  AHBENR,   0x14
.equ  APB2ENR,  0x18
.equ  APB1ENR,  0x1C
.equ  GPIOFEN,  0x00400000
.equ  GPIOEEN,  0x00200000
.equ  GPIODEN,  0x00100000
.equ  GPIOCEN,  0x00080000
.equ  GPIOBEN,  0x00040000
.equ  GPIOAEN,  0x00020000

// GPIO base addresses
.equ  GPIOA,    0x48000000
.equ  GPIOB,    0x48000400
.equ  GPIOC,    0x48000800
.equ  GPIOD,    0x48000c00
.equ  GPIOE,    0x48001000
.equ  GPIOF,    0x48001400

// GPIO Registers
.equ  MODER,    0x00
.equ  OTYPER,   0x04
.equ  OSPEEDR,  0x08
.equ  PUPDR,    0x0c
.equ  IDR,      0x10
.equ  ODR,      0x14
.equ  BSRR,     0x18
.equ  LCKR,     0x1c
.equ  AFRL,     0x20
.equ  AFRH,     0x24
.equ  BRR,      0x28

// Timer base addresses
.equ  TIM1,  0x40012c00
.equ  TIM2,  0x40000000
.equ  TIM3,  0x40000400
.equ  TIM6,  0x40001000
.equ  TIM7,  0x40001400
.equ  TIM14, 0x40002000
.equ  TIM15, 0x40014000
.equ  TIM16, 0x40014400
.equ  TIM17, 0x40014800

.equ  TIM_CR1,   0x00
.equ  TIM_CR2,   0x04
.equ  TIM_DIER,  0x0c
.equ  TIM_SR,    0x10
.equ  TIM_EGR,   0x14
.equ  TIM_CNT,   0x24
.equ  TIM_PSC,   0x28
.equ  TIM_ARR,   0x2c

// Timer configuration register bits
.equ TIM_CR1_CEN,  1<<0
.equ TIM_DIER_UDE, 1<<8
.equ TIM_DIER_UIE, 1<<0
.equ TIM_SR_UIF,   1<<0

// NVIC configuration registers
.equ NVIC, 0xe000e000
.equ ISER, 0x100
.equ ICER, 0x180


// You will need to add your own configuration symbols as needed.
.global setup_pins
setup_pins:
	push {r4, r5, r6, r7, lr}
	ldr r0, =RCC
	ldr r1, [r0, #AHBENR]
	ldr r2, =GPIOBEN
	ldr r3, =GPIOCEN
	ldr r4, =GPIOFEN
	orrs r1, r2
	orrs r1, r3
	orrs r1, r4
	str r1, [r0, #AHBENR]
	//sets up the RCC CLK
	ldr r0, =GPIOB
	ldr r1, [r0, #MODER]
	ldr r2, =0xff00
	bics r1, r2
	str r1, [r0, #MODER]
	//sets up  PB4, PB5, PB6, PB7 as inputs
	ldr r0, =GPIOF
	ldr r1, [r0, #MODER]
	ldr r2, =0x3cc0cf00
	bics r1, r2
	str r1, [r0, #MODER]
	//PF4, PF5, PF7, PF11, PF13, PF14 as inputs
	ldr r0, =GPIOB
	ldr r1, [r0, #MODER]
	ldr r2, =0xff
	bics r1, r2
	ldr r2, =0x55
	orrs r1, r2
	str r1, [r0, #MODER]
	//sets up PB0, PB1, PB2, PB3 as outputs
	ldr r0, =GPIOC
	ldr r1, [r0, #MODER]
	ldr r2, =0x3fffff
	bics r1, r2
	ldr r2, =0x155555
	orrs r1, r2
	str r1, [r0, #MODER]
	//PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, as outputs

	ldr r0, =GPIOF
	ldr r1, [r0, #MODER]
	ldr r2, = 0xc300003c
	bics r1, r2
	ldr r2, =0x41000014
	orrs r1, r2
	str r1, [r0, #MODER]
	//PF1, PF2, PF12, PF15 as outputs

	ldr r0, =GPIOF
	ldr r1, [r0, #PUPDR]
	ldr r2, =0x0cc00c00
	bics r1, r2
	ldr r2, = 0x04400400
	orrs r1, r2
	str r1, [r0, #PUPDR]
	//Configure the following pins to be pulled up: PF5, PF11, PF13
	ldr r0, =GPIOB
	ldr r1, [r0, #PUPDR]
	ldr r2, =0xff
	bics r1, r2
	ldr r2, =0xaa00
	orrs r1, r2
	str r1, [r0, #PUPDR]

	//Configure the following pins to be pulled down: PB4, PB5, PB6, PB7
	ldr r0, =GPIOF
	ldr r1, [r0, #PUPDR]
	ldr r2, = 0x3000c300
	bics r1, r2
	ldr r2, =0x20008200
	orrs r1, r2
	str r1, [r0, #PUPDR]
	//Configure the following pins to be pulled down: PF4, PF7, PF14
	ldr r0, =GPIOB
	ldr r1, [r0, #ODR]
	ldr r2, =0x1
	orrs r1, r2
	str r1, [r0, #ODR]
	//Write the (binary) value 0001 to the ODR of Port B.
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =0x0
	orrs r1, r2
	str r1, [r0, #ODR]
	//Write 0 to the ODR of Port C.
	pop  {r4, r5, r6, r7, pc}

.global setup_timer
setup_timer:
	push {r4, r5, r6, r7, lr}

	ldr r0, =RCC
	ldr r1, [r0, #APB2ENR]
	ldr r2, =0x20000
	orrs r1, r2
	str r1, [r0, #APB2ENR]
	//sets up clk for tim16


	ldr r0, =TIM16
	ldr r1, =2000
	str r1,[r0,#TIM_PSC]
	ldr r1,=2457
	str r1,[r0,#TIM_ARR]

	//set psc
	ldr r0,=TIM16
	ldr r1,[r0,#TIM_DIER]
	ldr r2,=TIM_DIER_UIE
	orrs r1,r2
	str r1,[r0,#TIM_DIER]
	//set dier
	ldr r0,=TIM16
	ldr r1,[r0,#TIM_CR1]
	ldr r2,=TIM_CR1_CEN
	orrs r1,r2
	str r1,[r0,#TIM_CR1]
	//set cr1
	ldr r0,=NVIC
	ldr r1,=ISER
	ldr r2,=(1<<21) //page 218 FRM
	str r2,[r0,r1]
	//set iser

	pop  {r4, r5, r6, r7, pc}




.global TIM16_IRQHandler
.type TIM16_IRQHandler, %function
TIM16_IRQHandler:
	push {r4, lr}

	ldr r0,=TIM16
	ldr r1,[r0,#TIM_SR] // read status reg
	ldr r2,=TIM_SR_UIF
	bics r1,r2 // turn off UIF
	str r1,[r0,#TIM_SR] // write it

	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	lsrs r2, r1, #8			//shift to get 10 9 8 next to eachother
	adds r2, #1
	lsls r3, r2, #8
	str r3, [r0, #ODR]

	ldr r0, =GPIOB
	ldr r1, [r0, #IDR]
	lsrs r2, r1, #4
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	orrs r1, r2
	str r1, [r0, #ODR]


	pop  {r4, pc}
.global main
main:
    bl setup_pins
    bl setup_timer
endless: // Do nothing else
    wfi
    b endless
