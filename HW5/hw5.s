.cpu cortex-m0
.thumb
.syntax unified
.fpu softvfp

.equ RCC,       0x40021000
.equ GPIOA,     0x48000000
.equ GPIOB,     0x48000400
.equ GPIOC,     0x48000800
.equ AHBENR,    0x14
.equ APB2ENR,   0x18
.equ APB1ENR,   0x1c
.equ IOPAEN,    0x20000
.equ IOPBEN,    0x40000
.equ IOPCEN,    0x80000
.equ SYSCFGCOMPEN, 1
.equ TIM3EN,    2
.equ MODER,     0x00
.equ OSPEEDR,   0x08
.equ PUPDR,     0x0c
.equ IDR,       0x10
.equ ODR,       0x14
.equ BSRR,      0x18
.equ BRR,       0x28
.equ PC8,       0x100

// SYSCFG control registers
.equ SYSCFG,    0x40010000
.equ EXTICR1,   0x08
.equ EXTICR2,   0x0c
.equ EXTICR3,   0x10
.equ EXTICR4,   0x14

// NVIC control registers
.equ NVIC,      0xe000e000
.equ ISER,      0x100

// External interrupt control registers
.equ EXTI,      0x40010400
.equ IMR,       0
.equ RTSR,      0x8
.equ PR,        0x14

.equ TIM3,      0x40000400
.equ TIMCR1,    0x00
.equ DIER,      0x0c
.equ TIMSR,     0x10
.equ PSC,       0x28
.equ ARR,       0x2c

// Popular interrupt numbers
.equ EXTI0_1_IRQn,   5
.equ EXTI2_3_IRQn,   6
.equ EXTI4_15_IRQn,  7
.equ EXTI4_15_IRQn,  7
.equ TIM2_IRQn,      15
.equ TIM3_IRQn,      16
.equ TIM6_DAC_IRQn,  17
.equ TIM7_IRQn,      18
.equ TIM14_IRQn,     19
.equ TIM15_IRQn,     20
.equ TIM16_IRQn,     21
.equ TIM17_IRQn,     22

//====================================================================
// Q1
//====================================================================
.global subfoo
subfoo:

	cmp r0, #2
	bhi subfoo_if
	beq subfoo_if
	// if (x < 2)
	movs r0, #0
	//return 0
	pop {r4, r5, pc}
subfoo_if:
	movs r4, r0
	movs r5, #1
	ands r4, r5
	cmp r4, #0
	bne return_shiftsubfoo
	// if ((x & 1) == 0)
	subs r0, #1
	bl subfoo
	adds r0, #2
	//return 2 + subfoo(x - 1);
	pop {r4, r5, pc}
return_shiftsubfoo:
	lsrs r0, #1
	bl subfoo
	adds r0, #1
	//return 1 + subfoo(x >> 1);

	pop {r4, r5, pc}

//====================================================================
// Q2
//====================================================================
.global enable_portb
enable_portb:
	push {r4, lr}
	ldr r0, =RCC			//load RCC
	ldr r1, [r0, #AHBENR]	//load RCC clk
	ldr r3, =0x00040000		//GPIOBEN
	orrs r1, r3				//update GPIOBEN into clk without changing other clk bits
	str r1, [r0, #AHBENR]	//stores
	pop {r4, pc}
//====================================================================
// Q3
//====================================================================
.global enable_portc
enable_portc:
	push {r4, lr}
	ldr r0, =RCC			//load RCC
	ldr r1, [r0, #AHBENR]	//load RCC clk
	ldr r3, =0x00080000		//load GPIOCEN
	orrs r1, r3				//update GPIOCEN into clk withouot changin other clk bits
	str r1, [r0, #AHBENR]	//stores
	pop {r4, pc}
//====================================================================
// Q4
//====================================================================
.global setup_pb3
setup_pb3:
	push {r4, lr}
 	ldr r0, =GPIOB
 	ldr r1, [r0, #MODER]	//moder gpioB register
 	ldr r2, =0xc0			// 1100 0000
 	bics r1, r2				// clears out pb3 of MODER register
 	str r1, [r0, #MODER]	// stores cleared pb3, 00 serves as an input
 	ldr r1, [r0, #PUPDR]	// PUPDR gpioB register
 	bics r1, r2				// clears pb3 of PUPDR
 	ldr r2, =0x80			// 1000 0000
 	orrs r1, r2				// sets byte 3 to be a pull DOWN resistor
  	str r1, [r0, #PUPDR]
  	pop {r4, pc}
//====================================================================
// Q5
//====================================================================
.global setup_pb4
setup_pb4:
	push {r4, lr}
 	ldr r0, =GPIOB
 	ldr r1, [r0, #MODER]	//moder gpioB register
 	ldr r2, =0x300			// 0011 0000 0000
 	bics r1, r2				// clears out byte 4
 	str r1, [r0, #MODER]	// stores xx00 xxxx xxxx
 	ldr r1, [r0, #PUPDR]	// pupdr gpioB register
 	bics r1, r2				// clears out byte 4
  	str r1, [r0, #PUPDR]	//
	pop {r4, pc}
//====================================================================
// Q6
//====================================================================
.global setup_pc8
setup_pc8:
	push {r4, lr}
 	ldr r0, =GPIOC
 	ldr r1, [r0, #MODER]
 	ldr r2, =0x30000		//PC8
 	bics r1, r2				//clear out PC8
 	ldr r2, = 0x10000		//load for output
 	orrs r1, r2
 	str r1, [r0, #MODER]

 	ldr r1, [r0, #OSPEEDR]
 	ldr r2, =0x30000
 	bics r1, r2
 	orrs r1, r2

  	str r1, [r0, #OSPEEDR]
  	pop {r4, pc}
//====================================================================
// Q7
//====================================================================
.global setup_pc9
setup_pc9:
	push {r4, lr}
 	ldr r0, =GPIOC
 	ldr r1, [r0, #MODER]
 	ldr r2, =0xc0000		//PC9
 	bics r1, r2				//clear pc 9
 	ldr r2, =0x40000		//set to output
 	orrs r1, r2
 	str r1, [r0, #MODER]

 	ldr r1, [r0, #OSPEEDR]
 	ldr r2, =0xc0000
 	bics r1, r2
 	ldr r2, =0x40000
 	orrs r1, r2
  	str r1, [r0, #OSPEEDR]
  	pop {r4, pc}
//====================================================================
// Q8
//====================================================================
.global action8
action8:
	push {r4, r5, r6, r7, lr}
	ldr r0, =GPIOB
	ldr r1, [r0, #IDR]
	movs r2, r1
	movs r7, #8
	ands r1, r7				//AND in order to get the PB3 bit alone
	lsrs r3, r1, #3			//PB3 shift down

	movs r7, #16
	ands r2, r7				//AND in order to get the PB4 bit alone
	lsrs r4, r2, #4			//pb4


	cmp r3, #1
	beq action8_if
	b action8_else

action8_if:
	 cmp r4, #0
	 bne action8_else

action8_set_PC8_0:
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =0x100
	bics r1, r2
	str r1, [r0, #ODR]
	pop  {r4, r5, r6, r7, pc}
action8_else:
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =0x100
	orrs r1, r2
	str r1, [r0, #ODR]
	pop  {r4, r5, r6, r7, pc}
//====================================================================
// Q9
//====================================================================
.global action9
action9:
	push {r4, r5, r6, r7, lr}
	ldr r0, =GPIOB
	ldr r1, [r0, #IDR]
	movs r2, r1
	movs r7, #8
	ands r1, r7
	lsrs r3, r1, #3			//PB3

	movs r7, #16
	ands r2, r7
	lsrs r4, r2, #4			//pb4


	cmp r3, #0
	beq action9_if
	b action9_else

action9_if:
	 cmp r4, #1
	 bne action9_else

action9_set_PC9_0:
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =0x200
	orrs r1, r2
	str r1, [r0, #ODR]
	pop  {r4, r5, r6, r7, pc}
action9_else:
	ldr r0, =GPIOC
	ldr r1, [r0, #ODR]
	ldr r2, =0x200
	bics r1, r2
	str r1, [r0, #ODR]
	pop  {r4, r5, r6, r7, pc}
//====================================================================
// Q10
//====================================================================
// Do everything needed to write the ISR here...
.global EXTI2_3_IRQHandler
.type EXTI2_3_IRQHandler, %function
EXTI2_3_IRQHandler:
	push {lr}				// why does bx lr not work
							// why do we not or
	ldr r0, =EXTI
	ldr r1, =0x14 			//EXTI_PR
	ldr r2, =1<<3
	ldr r3, [r0, r1]
	str r2, [r0, r1]
	ldr r0, =counter
	ldr r1, [r0]
	adds r1, #1
	str r1, [r0]
	pop {pc}
//====================================================================
// Q11
//====================================================================
.global enable_exti
enable_exti:
push {r4, r5, r6, lr}
	// Student code goes below
	ldr r0, =RCC
	ldr r2, [r0, #APB2ENR]
	ldr r1, =SYSCFGCOMPEN
	orrs r1, r2
	str r1, [r0, #APB2ENR]			//table 7.4.15 for RCC register map
	ldr r0, =SYSCFG

	ldr r1, [r0, #EXTICR1]
	ldr r2, =0x700					//pg 177 of FRM
	bics r1, r2
	ldr r2, =0x100					// x001: PB[x] pin
	orrs r1, r2
	str r1, [r0, #EXTICR1]

	ldr r0, =EXTI					//rising edge
	ldr r1, =RTSR
	ldr r2, [r0, r1]
	ldr r3, =1<<2
	orrs r2, r3
	str r2, [r0, r1]

	ldr r1, =IMR					//unmask
	ldr r0, =EXTI
	ldr r2, [r0, r1]
	ldr r3, =1<<2
	orrs r2, r3
	str r2, [r0, r1]

	ldr r2, = 1<<EXTI2_3_IRQn		//enable ISER
	ldr  r0,=NVIC
    ldr  r1,=ISER
    str  r2,[r0,r1]
	//unmasks interrupt

	// Student code goes above
	pop  {r4, r5, r6, pc}
//====================================================================
// Q12   TIM3_IRQHandler

//====================================================================
// Do everything needed to write the ISR here...
.global TIM3_IRQHandler
.type TIM3_IRQHandler, %function
TIM3_IRQHandler:
	push {lr}
	//always turn off
	ldr r0,=TIM3
	ldr r1,[r0,#TIMSR] // read status reg
	ldr r2,=1<<0		//TIM_SR_UIF
	bics r1,r2 // turn off UIF
	str r1,[r0,#TIMSR] // write it
	// clears the UIF bit

	//until you acknowledge the interrupt it will be continuously be reinvoked
	//it will still be raised
	//need to clear UIF bit of TIM3 status register
	//the bit is cleared by writing a 0 to it

	movs r4, #1
	movs r0, #9
	lsls r4, r0
	ldr r3, =GPIOC
    ldr r2, [r3, #ODR]
	eors r2, r4
    str r2, [r3, #ODR]





	pop {pc}
//====================================================================
// Q13
//====================================================================
.global enable_tim3
enable_tim3:
	push {lr}
	// Student code goes below
	//enable the clock to TIM7 in RCC_APB1ENR



	ldr r0, =RCC					//pg142 FRM
	ldr r1, [r0, #APB1ENR]
	ldr r2, =0x2						//TIM7EN
	orrs r1, r2
	str r1, [r0, #APB1ENR]			//clears out register for RCC CLK and updates tim7 clk
	//init timer for PSC, ARR, DIER, CR1
	ldr r0,=TIM3
	ldr r1,=4000-1
	str r1,[r0,#PSC]
	ldr r1,=3000-1
	str r1,[r0,#ARR]

	ldr r0,=TIM3
	ldr r1,[r0,#DIER]
	ldr r2,=1<<0					//TIM_DIER_UIE
	orrs r1,r2
	str r1,[r0,#DIER]

	ldr r0,=TIM3
	ldr r1,[r0,#TIMCR1]
	ldr r2,= 1<<0					//TIM_CR1_CEN
	orrs r1,r2
	str r1,[r0,#TIMCR1]				// enable intr gen
	//enables  the timer
	ldr r0,=NVIC
	ldr r1,=ISER
	ldr r2,=(1<<TIM3_IRQn)
	str r2,[r0,r1]					//unmasks the interrupt
	//invokes the IRQ handler of tim7



	// Student code goes above
	// when timer is enabled, it will
	// generate an update event which triggers an
	// interrupt. The interrupt will remain pending
	// until the interrupt is unmasked. Then the ISR
	// will be invoked, and the pending bit will be
	// cleared.


	pop  {pc}















