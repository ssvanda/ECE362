//=============================================================================
// ECE 362 lab experiment 6 -- Analog Input/Output
//=============================================================================

#include "stm32f0xx.h"
#include <math.h>

// Be sure to change this to your login...
const char login[] = "ssvanda";

void internal_clock(void);
void display_float(float);
void control(void);

//============================================================================
// setup_adc()    (Autotest #1)
// Configure the ADC peripheral and analog input pins.
// Parameters: none
//============================================================================
void setup_adc(void)
{
    // GPIO Mode :
    // 00 =Digital input, 01 = Digital output,
    // 10 =Alternate function, 11 = Analog(default)

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          //Enable the clock to GPIO Port A
    GPIOA->MODER |= 0b111100;                     //sets ADC_IN1 and ADC_IN2 to be analog mode
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;         //enable clock for ADC unit
    RCC->CR2 |= RCC_CR2_HSI14ON;                //TURN ON hi-spd internal 14Mhz clk
    ADC1->CR |= ADC_CR_ADEN;                    //enable adc
    while(!(ADC1->ISR & ADC_ISR_ADRDY));        //wait for ADC to be ready
    while((ADC1->CR & ADC_CR_ADSTART));         //wait for ADC start to be 0
}

//============================================================================
// start_adc_channel()    (Autotest #2)
// Select an ADC channel, and initiate an A-to-D conversion.
// Parameters: n: channel number
//============================================================================
void start_adc_channel(int n)
{
    ADC1->CHSELR = 0;
    ADC1->CHSELR = 1<<n;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));        //wait for ADC to be ready
    ADC1->CR |= ADC_CR_ADSTART;                 //Set the ADSTART bit in the CR register
}

//============================================================================
// read_adc()    (Autotest #3)
// Wait for A-to-D conversion to complete, and return the result.
// Parameters: none
// Return value: converted result
//============================================================================
int read_adc(void)
{
    while (!(ADC1->ISR & ADC_ISR_EOC));         //Wait for the EOC bit to be set in the ADC's ISR register

    return  ADC1->DR;                           //Return the value read from the ADC's DR register
}

//============================================================================
// setup_dac()    (Autotest #4)
// Configure the DAC peripheral and analog output pin.
// Parameters: none
//============================================================================
void setup_dac(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          //Enable the clock to GPIO Port A
    GPIOA->MODER |= 0b1100000000;               //Change the configuration only for pin 4 for analog operation
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;          //enable clk to dac
    DAC->CR |= DAC_CR_TEN1;                     //enable trigger
    DAC->CR |= DAC_CR_TSEL1;                    //all ones, select software trigger
    DAC->CR |= DAC_CR_EN1;                      // enable dac channel 1

}

//============================================================================
// write_dac()    (Autotest #5)
// Write a sample to the right-aligned 12-bit DHR, and trigger conversion.
// Parameters: sample: value to write to the DHR
//============================================================================
void write_dac(int sample)
{
//    for(;;) {
        //wait for DAC to clear SWTRIG1 bit
        while((DAC->SWTRIGR & DAC_SWTRIGR_SWTRIG1) == DAC_SWTRIGR_SWTRIG1);
        //Write the integer parameter to the 12-bit right-aligned data holding register (DHR) of the DAC.
        DAC->DHR12R1 = sample;
        //Set the software trigger bit to initiate the conversion.
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
 //       sample = (sample + 1) & 0xfff;

 //   }



}


//============================================================================
// Parameters for the wavetable size and expected DAC rate.
//============================================================================
#define N 1000
#define RATE 20000
short int wavetable[N];

//============================================================================
// init_wavetable()    (Autotest #6)
// Write the pattern for one complete cycle of a sine wave into the
// wavetable[] array.
// Parameters: none
//============================================================================
void init_wavetable(void)
{
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// Global variables used for four-channel synthesis.
//============================================================================
int volume = 2048;
int stepa = 0;
int stepb = 0;
int stepc = 0;
int stepd = 0;
int offseta = 0;
int offsetb = 0;
int offsetc = 0;
int offsetd = 0;

//============================================================================
// set_freq_n()    (Autotest #7)
// Set the four step and four offset variables based on the frequency.
// Parameters: f: The floating-point frequency desired.
//============================================================================
void set_freq_a(float f)
{

    stepa = f * N / RATE * (1<<16); // B above middle C (493.883 Hz)
    if (f == 0) {
        stepa = 0;
        offseta = 0;
    }
}

void set_freq_b(float f)
{
    stepb = f * N / RATE * (1<<16); // B above middle C (493.883 Hz)
    if (f == 0) {
         stepb = 0;
         offsetb = 0;
    }
}

void set_freq_c(float f)
{
    stepc = f * N / RATE * (1<<16); // B above middle C (493.883 Hz)
    if (f == 0) {
         stepc = 0;
         offsetc = 0;
    }
}

void set_freq_d(float f)
{
    stepd = f * N / RATE * (1<<16); // B above middle C (493.883 Hz)
    if (f == 0) {
         stepd = 0;
         offsetd = 0;
    }
}

//============================================================================
// Timer 6 ISR    (Autotest #8)
// The ISR for Timer 6 which computes the DAC samples.
// Parameters: none
// (Write the entire subroutine below.)
//============================================================================

void TIM6_DAC_IRQHandler(void) {
//Acknowledge the timer interrupt by writing a zero to the UIF bit of the timer's status register.
    TIM6->SR &= ~(1<<0);            //clears UIF bit
//    TIM_SR_UIF
//Trigger the DAC. Do this first to avoid variable-delay calculations, below, from introducing any jitter into the signal.
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; // trigger dac
//Add stepn to offsetn.
    offseta = stepa + offseta;
    offsetb = stepa + offsetb;
    offsetc = stepa + offsetc;
    offsetd = stepa + offsetd;
//If any of the offsetn variables are greater than or equal to the maximum fixed-point value, subtract the maximum fixed-point value from it.
//change offsets to floats
//compare the fixed floating points to N
    if((offseta>>16) >= N) {
        offseta = offseta - (N<<16);
//        offseta = offseta * (1<<16);
    }
    if((offsetb>>16) >= N) {
        offsetb = offsetb - (N<<16);
//        offsetb = offsetb * (1<<16);
    }
    if((offsetc>>16) >= N) {
        offsetc = offsetc - (N<<16);
//        offsetc = offsetc * (1<<16);
    }
    if((offsetd>>16) >= N) {
        offsetd = offsetd - (N<<16);
//        offsetd = offsetd * (1<<16);
    }
    int wavetableA = wavetable[offseta>>16];
    int wavetableB = wavetable[offsetb>>16];
    int wavetableC = wavetable[offsetc>>16];
    int wavetableD = wavetable[offsetd>>16];
    int variable;
//Look up the four samples at each of the four offsets in the wavetable array and add them together into a single combined sample variable.
    variable = wavetableA + wavetableB + wavetableC + wavetableD;
//Reduce and shift the combined sample to the range of the DAC by shifting it right by 5 (dividing it by 32) and then adding 2048.
    variable = (variable>>5) + 2048;
//If the adjusted sample is greater than 4095, set it to 4095. (Clip a too-high value.)
    if (variable > 4095) {
        variable = 4095;
    }
//If the adjusted sample is less than 0, set it to 0. (Clip a too-low value.)
    if (variable < 0) {
        variable = 0;
    }
//Write the final adjusted sample to the DAC's DHR12R1 register.
    DAC->DHR12R1 = variable;
}




//============================================================================
// setup_tim6()    (Autotest #9)
// Configure Timer 6 to raise an interrupt RATE times per second.
// Parameters: none
//============================================================================
void setup_tim6(void)
{
//Enable the RCC clock to Timer 6
    RCC->APB1ENR |= (0b10000);
//    RCC_APB1ENR_TIM6EN
//Set the Timer 6 PSC and ARR so that an update event occurs exactly 20000 times per second
    TIM6->PSC = 480-1;
    TIM6->ARR = 5-1;
    //20000
    TIM6->DIER |=(1<<0);
    TIM6->CR1 |= (1<<0);
    NVIC->ISER[0] = (1<<TIM6_DAC_IRQn);
}

int main(void)
{
//    internal_clock(); // Use the internal oscillator if you need it
    autotest(); // test all of the subroutines you wrote
    init_wavetable();
    setup_dac();
    setup_adc();
    setup_tim6();
    set_freq_a(261.626); // Middle 'C'
    set_freq_b(329.628); // The 'E' above middle 'C'
    control();
    while(1) {
        for(int out=0; out<4096; out++) {
            if ((TIM6->CR1 & TIM_CR1_CEN) == 0)
                write_dac(out);
            start_adc_channel(0);
            int sample = read_adc();
            float level = 2.95 * sample / 4095;
            display_float(level);
        }
    }
}
