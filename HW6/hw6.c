#include "stm32f0xx.h"

    // That's it.  The #include is all you need to start.

unsigned int subfoo(unsigned int);
void enable_portb(void);
void enable_portc(void);
void setup_pb3(void);
void setup_pb4(void);
void setup_pc8(void);
void setup_pc9(void);
void action8(void);
void EXTI2_3_IRQHandler(void); //isr
void enable_exti(void);
void TIM3_IRQHandler(void);
void enable_tim3(void);


unsigned int subfoo(unsigned int x) {
    if (x < 2)
        return 0;
    if ((x & 1) == 0)
        return 2 + subfoo(x - 1);
    return 1 + subfoo(x >> 1);
}

void enable_portb(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

void enable_portc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
}

void setup_pb3(void) {
    GPIOB->MODER &= ~(0xc0);
    GPIOB->PUPDR &= ~(0xc0);
    GPIOB->PUPDR |=  (0x80);

}

void setup_pb4(void) {
    GPIOB->MODER &= ~(0x300);
    GPIOB->PUPDR &= ~(0x300);
}

void setup_pc8(void) {
    GPIOC->MODER &= ~(0x30000);
    GPIOC->MODER |=  (0x10000);
    GPIOB->OSPEEDR &= ~(0x30000);
    GPIOC->OSPEEDR |=  (0x30000);
}

void setup_pc9(void) {
    GPIOC->MODER &= ~(0xc0000);
    GPIOC->MODER |=  (0x40000);
    GPIOC->OSPEEDR &= ~(0xc0000);
    GPIOC->OSPEEDR |=  (0x40000);
}

void action8(void) {
    if  (((GPIOB->IDR >> 3) == 1) && ((GPIOB->IDR >> 4) == 0))
        GPIOC->ODR &= ~(0x100);
    else
        GPIOC->ODR |=  (0x100);
}

void action9(void) {
//   int a = (((GPIOB->IDR) >> 3) == 0);
//   int b = (((GPIOB->IDR) >> 4) == 1);
//    if  (a && b)
//        GPIOC->ODR |=  (0x200);
//    else
//        GPIOC->ODR &= ~(0x200);

//    if  (((GPIOB->IDR >> 3) == 0) && ((GPIOB->IDR >> 4) == 1))
//        GPIOC->ODR |= (0x200);
//    else
//        GPIOC->ODR &= ~(0x200);
//    if  (((GPIOB->IDR >> 3) == 0) && ((GPIOB->IDR >> 4) == 1))
//        GPIOC->BSRR =  (0x200);
//    else
//        GPIOC->BRR = (0x200);
    int a = (GPIOB->IDR);
    int b = a & 0x10;
    int c = a & 0x8;
    if ((b  && !c))
        GPIOC->ODR |= (0x200);
    else
        GPIOC->ODR &= ~(0x200);


}

void EXTI2_3_IRQHandler(void) {
    extern volatile int counter;

    EXTI->PR |= (1<<3);
    counter = counter + 1;
}


void enable_exti(void) {
    RCC->APB2ENR |= (0x1);
//    SYSCFG->EXTICR

    SYSCFG->EXTICR[0] &= ~(0x700); //////////////////////
    SYSCFG->EXTICR[0] |=  (0x100);/////////////////////////

    EXTI->RTSR |= (1<<2);

    EXTI->IMR |= (1<<2);

    NVIC->ISER[0] |= (1<<6);
}

void TIM3_IRQHandler(void) {
    TIM3->SR &= ~(1<<0);

    GPIOC->ODR ^= (1<<9);


}


void enable_tim3(void) {
    RCC->APB1ENR |= (0x2);

    TIM3->PSC = 4000-1;
    TIM3->ARR = 3000-1;
    TIM3->DIER |=(1<<0);
    TIM3->CR1 |= (1<<0);
    NVIC->ISER[0] = (1<<16);              /////////////////////////

}




















