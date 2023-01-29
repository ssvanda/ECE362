#include "stm32f0xx.h"
#include <string.h>

    int hrs = 12;
    int min = 06;
    int sec = 30;
    int eighth;

const char font[] = {
        [' '] = 0x00,
        ['0'] = 0x3f,
        ['1'] = 0x06,
        ['2'] = 0x5b,
        ['3'] = 0x4f,
        ['4'] = 0x66,
        ['5'] = 0x6d,
        ['6'] = 0x7d,
        ['7'] = 0x07,
        ['8'] = 0x7f,
        ['9'] = 0x67,
        ['A'] = 0x77,
        ['B'] = 0x7c,
        ['C'] = 0x39,
        ['D'] = 0x5e,
        ['*'] = 0x49,
        ['#'] = 0x76,
        ['.'] = 0x80,
        ['?'] = 0x53,
        ['b'] = 0x7c,
        ['r'] = 0x50,
        ['g'] = 0x6f,
        ['i'] = 0x10,
        ['n'] = 0x54,
        ['u'] = 0x1c,
};

uint16_t digit[8*4];

void set_digit(int n, char c)
{
    digit[n] = (n<<8) | font[c];
}

void set_string(const char *s)
{
    for(int n=0; s[n] != '\0'; n++)
        set_digit(n,s[n]);
}

int main(void)
{

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= 0x155555;
    set_string("running.");

    // display loop
//    for(;;) {
//        for(int x=0; x < 8; x++) {
//            GPIOC->ODR = digit[x];
//            for(int n=0; n < 100; n++);
//        }
//    }

    GPIOC->ODR &= ~(0b1111111111111111);
    GPIOC->IDR &= ~(0b1111111111111111);
    GPIOC->ODR |= 0b11111111;
    GPIOC->IDR |= 0b11100000000;


    RCC->AHBENR |= RCC_AHBENR_DMA1EN; //Enable the RCC clock to the DMA controller
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; //turn off DMA
    DMA1_Channel1->CMAR = (uint32_t)digit; //Set CMAR to the address of the beginning of the digit array
    DMA1_Channel1->CPAR = (uint32_t) & (GPIOC->ODR); //Set CPAR to the address of the Port C ODR. Remember to cast it.



    DMA1_Channel1->CNDTR = 8*4;   //Set CNDTR to 8 to indicate there are 8 things to transfer.
    DMA1_Channel1->CCR |= DMA_CCR_DIR; //Set the DIR bit to indicate the direction of copying is from the "memory" address to the "peripheral" address.
//    DMA1_Channel1->CCR = DMA_CCR_TCIE;
//    DMA1_Channel1->CCR &= ~DMA_CCR_HTIE;
    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE; //Clear the memory datum size
    DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE; //Clear the peripheral datum size

    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; //Set the memory datum size (in the MSIZE field) to 16 bits.
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; //Set the peripheral datum size (in the PSIZE field) to 16 bits.

    DMA1_Channel1->CCR |= DMA_CCR_MINC; //Set the MINC bit so that the memory address is incremented by 2 after each transfer.
//    DMA1_Channel1->CCR |= DMA_CCR_PINC;
//    DMA1_Channel1->CCR |= DMA_CCR_MEM2MEM;
//    DMA1_Channel1->CCR &= ~DMA_CCR_PL;
    DMA1_Channel1->CCR |= DMA_CCR_CIRC; //Set the CIRC bit so that, once all 8 transfers complete, the entire transaction is restarted, over and over again.
    DMA1_Channel1->CCR |= DMA_CCR_EN;//Last: Set the EN bit to enable the DMA channel.



    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; //Turn on the RCC clock to TIM17
//configure the PSC and ARR to have an update event at a rate of 1000 Hz
    TIM17->PSC = 20-1;
    TIM17->ARR = 20-1;
// setting the UIE bit in the TIM17 DIER
// set only the UDE bit in the TIM17 DIER
    TIM17->DIER |=(TIM_DIER_UDE);

    TIM17->CR1 |= (1<<0); //set the CEN bit of TIM17 CR1 to let the counter run
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// set up a second timer to invoke an ISR with each update
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //enable the RCC clock to the selected timer
// configure it to have an update event 8 times per second
    TIM6->PSC = 48000-1;
    TIM6->ARR = 125-1;
    TIM6->DIER |=(1<<0);
    TIM6->CR1 |= (1<<0);
    NVIC->ISER[0] = (1<<TIM6_DAC_IRQn);


}
void TIM6_DAC_IRQHandler(void) {

    TIM6->SR &= ~(TIM_SR_UIF);           //clears UIF bit
    eighth += 1;
        if (eighth >= 8) { eighth -= 8; sec += 1; }
        if (sec >= 60)   { sec -= 60;   min += 1; }
        if (min >= 60)   { min -= 60;   hrs += 1; }
        if (hrs >= 24)   { hrs -= 24; }
        char time[8];
        sprintf(time, "%02d%02d%02d  ", hrs, min, sec);
        set_string(time);

        if (eighth > 0 && eighth < 4) {
               memcpy(&digit[8*eighth], digit, 2*8);
           }

}

/*
void TIM17_IRQHandler(void) {
    TIM17->SR &= ~(TIM_SR_UIF);
}
*/
