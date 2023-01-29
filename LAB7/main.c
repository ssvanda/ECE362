
//===========================================================================
// ECE 362 lab experiment 7 -- Pulse-Width Modulation
//===========================================================================

#include "stm32f0xx.h"
#include <string.h>  // for memset() declaration
#include <math.h>    // for M_PI

// Be sure to change this to your login...
const char login[] = "ssvanda";

//===========================================================================
// setup_tim1()    (Autotest #1)
// Configure Timer 1 and its PWM output pins.
// Parameters: none
//===========================================================================
void setup_tim1()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          //Activate the RCC clock to GPIO Port A.
    GPIOA->MODER &= ~(0xff0000);//clear MODER
    GPIOA->MODER |=   0b101010100000000000000000;   //Configure the MODER for the four pins to set them for alternate function use.
    GPIOA->AFR[1] &= ~(0xffff);      //clears afr register
    GPIOA->AFR[1] |= (0x2222);       //second bit of each AFR register from 8-11
//    TIM1->BDTR &= ~TIM_BDTR_MOE;
    RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
    TIM1->BDTR |= TIM_BDTR_MOE;       //enable the MOE bit of the break and dead-time register (BDTR)
//    TIM1->BDTR |= 0b1000000000000000;       //enable the MOE bit of the break and dead-time register (BDTR)


    TIM1->PSC = 1-1;
    TIM1->ARR = 2400-1;
// Configure the "capture/compare mode registers", CCMR1 and CCMR2, to
// set channels 1, 2, 3, and 4 for PWM mode 1. There are two 3-bit fields in
// each register that must be adjusted to accomplish this.
    //TIM1->CCMR1 &= ~(0xffff);
    //TIM1->CCMR1 &= ~(0xffff);
    //TIM1->CCMR2 &= ~(0xffff);
    //TIM1->CCMR2 &= ~(0xffff);

    TIM1->CCMR1 |= (0b1100000);
    TIM1->CCMR1 |= (0b110000000000000);
    TIM1->CCMR2 |= (0b1100000);
    TIM1->CCMR2 |= (0b110000000000000);

// Configure the CCMR2 register to set the "preload enable" bit only for channel 4.
    TIM1->CCMR2 |= (TIM_CCMR2_OC4PE);

// Enable the (uninverted) channel outputs for all four channels by turning on
// the CC1E, CC2E, etc. bits for in the "capture/compare enable register",
// CCER. Until you do this for each channel, the timer will not affect the outputs.
    TIM1->CCER |= (TIM_CCER_CC1E); //TIM_CCER_CC1E
    TIM1->CCER |= (TIM_CCER_CC2E);
    TIM1->CCER |= (TIM_CCER_CC3E);
    TIM1->CCER |= (TIM_CCER_CC4E);


    //using tim1 as background timer so dont call it to do things
    //TIM1->DIER |=(1<<0);
    TIM1->CR1 |= (1<<0);
    //NVIC->ISER[0] = (1<<TIM1_BRK_UP_TRG_COM_IRQn);

}

//===========================================================================
// Parameters for the wavtable size and expected synthesis rate.
//===========================================================================
#define N 1000
#define RATE 20000
short int wavetable[N];

//===========================================================================
// init_wavetable()    (Autotest #2)
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
// Parameters: none
//===========================================================================
void init_wavetable(void)
{
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);

}

//===========================================================================
// The global variables used for four-channel synthesis.
//===========================================================================
int volume = 2048;
int stepa = 0;
int stepb = 0;
int stepc = 0;
int offseta = 0;
int offsetb = 0;
int offsetc = 0;

//===========================================================================
// set_freq_n()    (Autotest #2)
// Set the three step and three offset variables based on the frequency.
// Parameters: f: The floating-point frequency desired.
//===========================================================================
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


//===========================================================================
// Timer 6 ISR    (Autotest #2)
// The ISR for Timer 6 computes the DAC samples.
// Parameters: none
// (Write the entire subroutine below.)
//===========================================================================
void TIM6_DAC_IRQHandler(void) {
//Acknowledge the timer interrupt by writing a zero to the UIF bit of the timer's status register.
    TIM6->SR &= ~(1<<0);            //clears UIF bit
//    TIM_SR_UIF
//Trigger the DAC. Do this first to avoid variable-delay calculations, below, from introducing any jitter into the signal.
    //DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; // trigger dac
//Add stepn to offsetn.
    offseta = stepa + offseta;
    offsetb = stepa + offsetb;
    offsetc = stepa + offsetc;
    //offsetd = stepa + offsetd;
//If any of the offsetn variables are greater than or equal to the maximum fixed-point value, subtract the maximum fixed-point value from it.
//change offsets to floats
//compare the fixed floating points to N
    if((offseta>>16) >= N) {
        offseta = offseta - (N<<16);

    }
    if((offsetb>>16) >= N) {
        offsetb = offsetb - (N<<16);

    }
    if((offsetc>>16) >= N) {
        offsetc = offsetc - (N<<16);

    }
    //if((offsetd>>16) >= N) {
    //    offsetd = offsetd - (N<<16);
    //}
    int wavetableA = wavetable[offseta>>16]; //sample a
    int wavetableB = wavetable[offsetb>>16]; //sample a
    int wavetableC = wavetable[offsetc>>16]; //sample a
    //int wavetableD = wavetable[offsetd>>16];
    int variable;
//Look up the four samples at each of the four offsets in the wavetable array and add them together into a single combined sample variable.
    variable = wavetableA + wavetableB + wavetableC;// + wavetableD; //sample
//Reduce and shift the combined sample to the range of the DAC by shifting it right by 5 (dividing it by 32) and then adding 2048.
    //variable = (variable>>5) + 2048;
    variable = ((variable * volume)>>17) + 1200;
//If the adjusted sample is greater than 4095, set it to 4095. (Clip a too-high value.)
    if (variable > 4095) {
        variable = 4095;
    }
//If the adjusted sample is less than 0, set it to 0. (Clip a too-low value.)
    if (variable < 0) {
        variable = 0;
    }
//Write the final adjusted sample to the DAC's DHR12R1 register.
    //DAC->DHR12R1 = variable;
    TIM1->CCR4 = variable;

}

//===========================================================================
// setup_tim6()    (Autotest #2)
// Set the three step and three offset variabls based on the frequency.
// Parameters: f: The floating-point frequency desired.
//===========================================================================
void setup_tim6()
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

//===========================================================================
// enable_ports()    (Autotest #3)
// Configure the pins for GPIO Ports B and C.
// Parameters: none
//===========================================================================
void enable_ports()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~(0b1111111111111111111111);
    GPIOC->MODER |=   0b0101010101010101010101;
    GPIOB->MODER &= ~(0b11111111);
    GPIOB->MODER |=   0b01010101;
    GPIOB->MODER &= ~(0b1111111100000000);
    GPIOB->PUPDR &= ~(0b1111111100000000);
    GPIOB->PUPDR |=   0b1010101000000000;


}

char offset;
char history[16];
char display[8];
char queue[2];
int  qin;
int  qout;

//===========================================================================
// show_digit()    (Autotest #4)
// Output a single digit on the seven-segmnt LED array.
// Parameters: none
//===========================================================================
void show_digit()
{
    int off = offset & 7;
    GPIOC->ODR = (off << 8) | display[off];
}

//===========================================================================
// set_row()    (Autotest #5)
// Set the row active on the keypad matrix.
// Parameters: none
//===========================================================================
void set_row()
{
    int row = offset & 3;
    GPIOB->BSRR = 0xf0000 | (1<<row);
}

//===========================================================================
// get_cols()    (Autotest #6)
// Read the column pins of the keypad matrix.
// Parameters: none
// Return value: The 4-bit value read from PC[7:4].
//===========================================================================
int get_cols()
{
    return (GPIOB->IDR >> 4) & 0xf;
}

//===========================================================================
// insert_queue()    (Autotest #7)
// Insert the key index number into the two-entry queue.
// Parameters: n: the key index number
//===========================================================================
void insert_queue(int n)
{
    n = (n | 0x80);
    queue[qin] = n;
    qin = !qin;


}

//===========================================================================
// update_hist()    (Autotest #8)
// Check the columns for a row of the keypad and update history values.
// If a history entry is updated to 0x01, insert it into the queue.
// Parameters: none
//===========================================================================
void update_hist(int cols)
{
    int row = offset & 3;
    for(int i=0; i < 4; i++) {
        history[4*row+i] = (history[4*row+i]<<1) + ((cols>>i)&1);
        if (history[4*row+i] == 0x1)
            insert_queue(4*row+i);
    }
}

//===========================================================================
// Timer 7 ISR()    (Autotest #9)
// This is the Timer 7 ISR
// Parameters: none
// (Write the entire subroutine below.)
//===========================================================================
void TIM7_IRQHandler(void) {

    TIM7->SR &= ~(TIM_SR_UIF);         ;   //clears UIF bit
    show_digit();
    int cols = get_cols();
    update_hist(cols);
    offset = (offset + 1) & 0x7; // count 0 ... 7 and repeat
    set_row();
}

//===========================================================================
// setup_tim7()    (Autotest #10)
// Configure timer 7
// Parameters: none
//===========================================================================
void setup_tim7()
{
    RCC->APB1ENR |= 0b100000; //RCC_APB1ENR_TIM7EN
    TIM7->PSC = 4800-1;
    TIM7->ARR = 10-1;
    TIM7->DIER |=(1<<0);
    TIM7->CR1 |= (1<<0);
    NVIC->ISER[0] = (1<<TIM7_IRQn);

}

//===========================================================================
// getkey()    (Autotest #11)
// Wait for an entry in the queue.  Translate it to ASCII.  Return it.
// Parameters: none
// Return value: The ASCII value of the button pressed.
//===========================================================================
int getkey()
{
    if (queue[qout] == 0)
        asm volatile("wfi");
    int copy = queue[qout];
    queue[qout] = 0;        //overwrite queue[qout] with zero
    qout = !qout;           // toggle the qout variable so it refers to the other queue element
    copy = copy & 0x7f;     // AND the saved copy of the queue element with 0x7f.

    if(copy == 13)
        return '0';
    if(copy == 0)
        return '1';
    if(copy == 1)
        return '2';
    if(copy == 2)
        return '3';
    if(copy == 4)
        return '4';
    if(copy == 5)
        return '5';
    if(copy == 6)
        return '6';
    if(copy == 8)
        return '7';
    if(copy == 9)
        return '8';
    if(copy == 10)
        return '9';
    if(copy == 3)
        return 'A';
    if(copy == 7)
        return 'B';
    if(copy == 11)
        return 'C';
    if(copy == 15)
        return 'D';
    if(copy == 14)
        return '#';
    if(copy == 12)
        return '*';
}

//===========================================================================
// This is a partial ASCII font for 7-segment displays.
// See how it is used below.
//===========================================================================
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

// Shift a new character into the display.
void shift(char c)
{
    memcpy(display, &display[1], 7);
    display[7] = font[c];
}

// Turn on the dot of the rightmost display element.
void dot()
{
    display[7] |= 0x80;
}

// Read an entire floating-point number.
float getfloat()
{
    int num = 0;
    int digits = 0;
    int decimal = 0;
    int enter = 0;
    memset(display,0,8);
    display[7] = font['0'];
    while(!enter) {
        int key = getkey();
        if (digits == 8) {
            if (key != '#')
                continue;
        }
        switch(key) {
        case '0':
            if (digits == 0)
                continue;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            num = num*10 + key-'0';
            decimal <<= 1;
            digits += 1;
            if (digits == 1)
                display[7] = font[key];
            else
                shift(key);
            break;
        case '*':
            if (decimal == 0) {
                decimal = 1;
                dot();
            }
            break;
        case '#':
            enter = 1;
            break;
        default: continue; // ABCD
        }
    }
    float f = num;
    while (decimal) {
        decimal >>= 1;
        if (decimal)
            f = f/10.0;
    }
    return f;
}

// Read a 6-digit BCD number for RGB components.
int getrgb()
{
    memset(display, 0, 8);
    display[0] = font['r'];
    display[1] = font['r'];
    display[2] = font['g'];
    display[3] = font['g'];
    display[4] = font['b'];
    display[5] = font['b'];
    int digits = 0;
    int rgb = 0;
    for(;;) {
        int key = getkey();
        if (key >= '0' || key <= '9') {
            display[digits] = font[key];
            digits += 1;
            rgb = (rgb << 4) + (key - '0');
        }
        if (digits == 6)
            break;
    }
    return rgb;
}

//==========================================================================
// setrgb()    (Autotest #12)
// Accept a BCD-encoded value for the 3 color components.
// Then update the CCR values appropriately.
// Parameters: rgb: the RGB color component values
//==========================================================================
void setrgb(int rgb)
{
    //NOT LOADED AS HEX VALUE, LOADED AS A BCD VALUE
    //need to shift to get each digit individually
    int red1 = ((rgb) & 0xf00000) >> 20;
    int red2 = ((rgb) & 0x0f0000) >> 16;
    int red = red1 *10 + red2;

    int green1= (rgb & 0x00f000) >> 12;
    int green2= (rgb & 0x000f00) >> 8;
    int green = green1*10 + green2;

    int blue1 = (rgb & 0x0000f0) >> 4;
    int blue2 = (rgb & 0x00000f);
    int blue  = blue1*10 + blue2;


    TIM1->CCR1 = (100 - red )*24; //arr = 2400
    TIM1->CCR2 = (100 - green)*24;
    TIM1->CCR3 = (100 - blue)*24;
}

void internal_clock();
void demo();
void autotest();

int main(void)
{
    internal_clock();
    demo();
    //autotest();
    enable_ports();
    init_wavetable();
    set_freq_a(261.626); // Middle 'C'
    set_freq_b(329.628); // The 'E' above middle 'C'
    set_freq_c(391.996); // The 'G' above middle 'C'
    setup_tim1();
    setup_tim6();
    setup_tim7();

    display[0] = font['r'];
    display[1] = font['u'];
    display[2] = font['n'];
    display[3] = font['n'];
    display[4] = font['i'];
    display[5] = font['n'];
    display[6] = font['g'];
    for(;;) {
        char key = getkey();
        if (key == 'A')
            set_freq_a(getfloat());
        else if (key == 'B')
            set_freq_b(getfloat());
        else if (key == 'C')
            set_freq_c(getfloat());
        else if (key == 'D')
            setrgb(getrgb());
    }
}
