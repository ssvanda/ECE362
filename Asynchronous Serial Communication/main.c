
//===========================================================================
// ECE 362 lab experiment 10 -- Asynchronous Serial Communication
//===========================================================================

#include "stm32f0xx.h"
#include "ff.h"
#include "diskio.h"
#include "fifo.h"
#include "tty.h"
#include <string.h> // for memset()
#include <stdio.h> // for printf()

void advance_fattime(void);
void command_shell(void);

// Write your subroutines below.

void setup_usart5(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;     //Enable the RCC clocks to GPIOC and GPIOD.
//Do all the steps necessary to configure pin PC12 to be routed to USART5_TX.
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |=  GPIO_MODER_MODER12_1;
    GPIOC->AFR[1] |= 0b0010<<(4*4);//check of datasheet page 43, need USART5_TX
                                   //the (x*4) is the AFRx, x corresponds to the pin number

//Do all the steps necessary to configure pin PD2 to be routed to USART5_RX.
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |=  GPIO_MODER_MODER2_1;
    GPIOD->AFR[0] |= 0b0010<<(2*4); //check of datasheet page 43, need USART5_RX
                                    //the (x*4) is the AFRx, x corresponds to the pin number

//Enable the RCC clock to the USART5 peripheral.
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
//Configure USART5 as follows:
    USART5->CR1 &= ~USART_CR1_UE;  //(First, disable it by turning off its UE bit.)
    USART5->CR1 &= ~USART_CR1_M;   //Set a word size of 8 bits.

    USART5->CR2 &= ~USART_CR2_STOP; //Set it for one stop bit.
    USART5->CR1 &= ~USART_CR1_PCE;  //Set it for no parity.
    USART5->CR1 &= ~USART_CR1_OVER8; //Use 16x oversampling.
    USART5->BRR = (48000000 / 115200); //Use a baud rate of 115200 (115.2 kbaud).
                                       //Use 16x oversampling.
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE; //Enable the transmitter and the receiver by setting the TE and RE bits..
    USART5->CR1 |= USART_CR1_UE; //Enable the USART.
//Finally, you should wait for the TE and RE bits to be acknowledged.
//This indicates that the USART is ready to transmit and receive.
    //while(~(USART5->CR1 & (USART_CR1_TE | USART_CR1_RE)));
    while(!(USART5->ISR & (USART_ISR_TEACK)));
    while(!(USART5->ISR & (USART_ISR_REACK)));

}
int simple_putchar(int argument) {
    while(!(USART5->ISR & USART_ISR_TXE)); //Wait for the USART5 ISR TXE to be set.
    USART5->TDR = argument; //Write the argument to the USART5 TDR (transmit data register).
    return argument; //Return the argument that was passed in

}

int simple_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE));
    return (USART5->RDR);
}
int __io_putchar(int ch) {
    return better_putchar(ch);
}

int __io_getchar(void) {
    return interrupt_getchar();
}

int better_putchar(int argument) {
    if (argument == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE)); //Wait for the USART5 ISR TXE to be set.
        USART5->TDR = '\r';
        while(!(USART5->ISR & USART_ISR_TXE)); //Wait for the USART5 ISR TXE to be set.
        USART5->TDR = '\n';
    }
    else {
        while(!(USART5->ISR & USART_ISR_TXE)); //Wait for the USART5 ISR TXE to be set.
        //when its set no transmission is going
        //when its not ready, it is in the process
        USART5->TDR = argument; //Write the argument to the USART5 TDR (transmit data register).
    }
    return argument; //Return the argument that was passed in

}

int better_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE));
    int variable = USART5->RDR;
    if (variable == '\r')
        return ('\n');
    else
        return (variable);
}

int interrupt_getchar(void) {
    while (fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi"); // wait for an interrupt
    }
    char ch = fifo_remove(&input_fifo);
    return ch;

}

int USART3_4_5_6_7_8_IRQHandler(void) {
    if (USART5->ISR & USART_ISR_ORE)        //Check and clear the ORE flag.
        USART5->ICR |= USART_ICR_ORECF;
    char ch = USART5->RDR;                  //Read the new character from the USART5 RDR.
    if (fifo_full(&input_fifo) == 1) {   //Check if the input_fifo is full. If it is,
                                            //return from the ISR. (Throw away the character.)
        return 0;
    }
    insert_echo_char(ch);           //Call insert_echo_char() with the character.
    return 0;

}
void enable_tty_interrupt(void) {
    while ((USART5->RDR));      //not neccessarily needed

    USART5->CR1 |= USART_CR1_RXNEIE;
    NVIC->ISER[0] = (1<<USART3_8_IRQn);
}

void setup_spi1(void) {
//Enable the RCC clock to GPIOA.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // 00: Input mode (reset state)
    // 01: General purpose output mode
    // 10: Alternate function mode
    // 11: Analog mode
//Configure PA12 to be a general-purpose output.
    GPIOA->MODER &= ~GPIO_MODER_MODER12;
    GPIOA->MODER |= GPIO_MODER_MODER12_0;
//Configure GPIOA so that pins 5, 6, and 7 are routed to SPI1.
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] &= ~(0b1111 << (5*4));
    GPIOA->AFR[0] &= ~(0b1111 << (6*4));
    GPIOA->AFR[0] &= ~(0b1111 << (7*4));
//Enable the RCC clock to the SPI1 peripheral.
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//Disable the SPI1 peripheral by turning off the SPE bit.
    SPI1->CR1 &= ~SPI_CR1_SPE;
//Set it for as low a baud rate as possible.
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR; //Set the baud rate as low as possible (maximum divisor for BR).
                                            //Configure the SPI channel to be in "master mode".
//Ensure that BIDIMODE and BIDIOE are cleared.
    SPI1->CR2 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);
//Set NSSP and configure the peripheral for an 8-bit word (which is the default).
    SPI1->CR2 |= SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
//Set the bit that sets the FIFO-reception threshold to 8-bits.
    SPI1->CR2 |= SPI_CR2_FRXTH;
//Enable the SPI1 peripheral.
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_high_speed(void) {
    SPI1->CR1 &= ~(SPI_CR1_SPE); //Disables the SPI1 SPE bit.
    SPI1->CR1 &= ~(SPI_CR1_BR); //Configure SPI1 for a 24 MHz SCK rate.
    SPI1->CR1 |= (SPI_CR1_BR_0); //Configure SPI1 for a 12 MHz SCK rate.
    SPI1->CR1 |=  (SPI_CR1_SPE);//Re-enable the SPI1 SPE bit.
}
void TIM14_IRQHandler(void) {
   TIM14->SR &= ~TIM_SR_UIF; //Acknowledge the interrupt.
   advance_fattime();
}
void setup_tim14(void){
//Create a subroutine named setup_tim14() that configures Timer 14 to raise an
//interrupt once every two seconds (0.5 Hz).
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 48000-1;
    TIM14->ARR = 2000-1;
    TIM14->DIER |=(1<<0);
    TIM14->CR1 |= (1<<0);
    NVIC->ISER[0] = (1<<TIM14_IRQn);
}

// Write your subroutines above.

/*int line_buffer_getchar(void) {
    USART_TypeDef *u = USART5;
    // If we missed reading some characters, clear the overrun flag.
    if (u->ISR & USART_ISR_ORE)
        u->ICR |= USART_ICR_ORECF;
    // Wait for a newline to complete the buffer.
    while(fifo_newline(&input_fifo) == 0) {
        while (!(u->ISR & USART_ISR_RXNE))
            ;
        insert_echo_char(u->RDR);
    }
    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}*/


const char testline[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789\r\n";

int main()
{
    setup_usart5();

    // Uncomment these when you're asked to...
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);

    // Test for 2.2 simple_putchar()
    //
/*
    for(;;)
        for(const char *t=testline; *t; t++)
            simple_putchar(*t);
*/

    // Test for 2.3 simple_getchar()

/*    for(;;)
        simple_putchar( simple_getchar() );*/

    // Test for 2.4 and 2.5 __io_putchar() and __io_getchar()

/*
    printf("Hello!\n");
    for(;;)
        putchar( getchar() );
*/

    // Test 2.6
    //
/*
    for(;;) {
        printf("Enter string: ");
        char line[100];
        fgets(line, 99, stdin);
        line[99] = '\0'; // just in case
        printf("You entered: %s", line);
    }
*/

    // Test for 2.7
    //
/*    enable_tty_interrupt();
    for(;;) {
        printf("Enter string: ");
        char line[100];
        fgets(line, 99, stdin);
        line[99] = '\0'; // just in case
        printf("You entered: %s", line);
    }*/

    // Test for 2.8 Test the command shell and clock
    //
    enable_tty_interrupt();
    setup_tim14();
    FATFS fs_storage;
    FATFS *fs = &fs_storage;
    f_mount(fs, "", 1);
    command_shell();

    return 0;
}
