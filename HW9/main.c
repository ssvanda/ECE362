#include <math.h>
#include "stm32f0xx.h"
#include <stdint.h>
#define SAMPLES 200
    uint16_t array[SAMPLES];


int main(void)  {
    RCC->AHBENR |=RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);
    GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1; //set to analog mode
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; //The RCC clock for the DAC must be enabled before making any changes.
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    RCC->AHBENR |= RCC_AHBENR_DMAEN; //Enable the RCC clock to the DMA controller
//configure the TRGO PAGE 500
    DAC->CR &= ~(DAC_CR_TSEL1);//setting the DMAEN1 bit of the DAC_CR register, each time the DAC is triggered
//Step 1.1 Configure the timer TRGO (trigger output)//////////////
    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_0;

//Step 1.2 Triggering DMA with the DAC
    DAC->CR |= DAC_CR_DMAEN1;

//Step 1.3 Triggering DMA with the timer
    TIM6->DIER |= TIM_DIER_UDE;
//Step 2: Create a waveform
    for(int x=0; x < SAMPLES; x += 1)
               array[x] = 2048 + 1952 * sin(2 * M_PI * x / SAMPLES);

//Step 3: Set up the timer

    TIM6->PSC = 1-1;
    TIM6->ARR = 12-1;
    //TIM6->DIER |=(1<<0);
    TIM6->CR1 |= (1<<0);
    TIM6->CR1 |= TIM_CR1_ARPE;//In your timer configuration, set the ARPE bit. This delays update of the value used for the ARR until the next update occurs. You'll see why that's needed below.


   //NVIC->ISER[0] = (1<<TIM6_DAC_IRQn);
//Step 4: Set up DMA///////////////////////////////////////////////////////////////
    DMA1_Channel3->CMAR = (uint32_t) array; //The memory source of data transfer is the array.
    DMA1_Channel3->CPAR = (uint32_t) & (DAC->DHR12R1); //The peripheral destination of data transfer is the DAC DHR12R1.
    DMA1_Channel3->CNDTR = SAMPLES; //The count of data elements to transfer is the number of elements in the array (200).
    DMA1_Channel3->CCR |= DMA_CCR_DIR; //Set the DIRection for copying from-memory-to-peripheral.
    DMA1_Channel3->CCR |= DMA_CCR_MINC;//The memory address should be incremented after every transfer.
    DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0; //Set the memory datum size to 16-bit.
    DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0; //Set the peripheral datum size to 16-bit.
    DMA1_Channel3->CCR |= DMA_CCR_CIRC; //The array will be transferred repeatedly, so the channel should be set for circular operation
    DMA1_Channel3->CCR |= DMA_CCR_EN; //Make sure you enable the DMA channel.
    DMA1_Channel3->CCR |= DMA_CCR_TCIE;
    NVIC->ISER[0] |= 1<<DMA1_Ch2_3_DMA2_Ch1_2_IRQn;
//Step 5: Set up the DAC//////////////////////////////////////////////////////////
    DAC->CR &= ~DAC_CR_TSEL1; //Page 283 //Set the TSEL1 field so that the DAC is triggered by the timer you selected.
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; // Make sure you enable the trigger for channel 1.
    DAC->CR |= DAC_CR_EN1;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; //If you're using the DAC to trigger the DMA channel, set the DMAEN1 bit.
    //for(;;);

// Step 7: Modify it

}

void DMA1_CH2_3_DMA2_CH1_2_IRQHandler(void) {
    //clears bit
    DMA1->IFCR |= (DMA_IFCR_CGIF3);
    if (TIM6->ARR == 11)
        TIM6->ARR = (12*4)-1;
    else
        TIM6->ARR = 12-1;

}


