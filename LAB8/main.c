
//===========================================================================
// ECE 362 lab experiment 8 -- SPI and DMA
//===========================================================================

#include "stm32f0xx.h"
#include "lcd.h"
#include <stdio.h> // for sprintf()

// Be sure to change this to your login...
const char login[] = "ssvanda";

// Prototypes for miscellaneous things in lcd.c
void nano_wait(unsigned int);

// Write your subroutines below
void small_delay(void) {
    nano_wait(1000000);
}
void bb_write_bit(int parameter) {
    GPIOB->ODR &= ~(1<<15);
    GPIOB->ODR |= parameter<<15; //Set the MOSI pin to the value of the parameter.
    small_delay();
    GPIOB->ODR &= ~(GPIO_ODR_13); //Set the SCK pin to high
    GPIOB->ODR |=  (GPIO_ODR_13); //Set the SCK pin to high
    small_delay();
    GPIOB->ODR &= ~(GPIO_ODR_13); //Set the SCK pin to low.

}

void bb_write_byte(int parameter) {
//If you're new at this, remember that you can use the >> operator to shift values
//to the left by an arbitrary amount. Then use the & operator to AND the result
//with a 1 to isolate one bit.
    bb_write_bit((parameter>>7) & 0x1);
    bb_write_bit((parameter>>6) & 0x1);
    bb_write_bit((parameter>>5) & 0x1);
    bb_write_bit((parameter>>4) & 0x1);
    bb_write_bit((parameter>>3) & 0x1);
    bb_write_bit((parameter>>2) & 0x1);
    bb_write_bit((parameter>>1) & 0x1);
    bb_write_bit((parameter>>0) & 0x1);

}
void bb_cmd(int parameter) {
    GPIOB->ODR &= ~(GPIO_ODR_12); //Set the NSS pin low to start an SPI transfer.
    small_delay();
    bb_write_bit(0); // RS is 0 to start a command.
    bb_write_bit(0); // R/W is 0 for a write.
    bb_write_byte(parameter); //with the parameter passed
    small_delay();
    GPIOB->ODR |= (GPIO_ODR_12); //Set the NSS pin high to signal the end of the SPI transfer.
    small_delay();
}
void bb_data(int parameter) {
    GPIOB->ODR &= ~(GPIO_ODR_12); //Set the NSS pin low to start an SPI transfer.
    small_delay();
    bb_write_bit(1); // RS is 1 to start a data byte, sends a data byte
    bb_write_bit(0); // R/W is 0 for a write.
    bb_write_byte(parameter); //with the parameter passed
    small_delay();
    GPIOB->ODR |= (GPIO_ODR_12); //Set the NSS pin high to signal the end of the SPI transfer.
    small_delay();
}
void bb_init_oled(void) {
    nano_wait(1000000); //Use nano_wait() to wait 1 ms for the display to power up and stabilize.
    bb_cmd(0x38); // set for 8-bit operation
    bb_cmd(0x08); // turn display off
    bb_cmd(0x01); // clear display
    nano_wait(2000000); //Use nano_wait() to wait 2 ms for the display to clear.
    bb_cmd(0x06); // set the display to scroll
    bb_cmd(0x02); // move the cursor to the home position
    bb_cmd(0x0c); // turn the display on

}
void bb_display1(const char *parameter ) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bb_cmd(0x02); // move the cursor to the home position
    //array
    //last character of string is null byte
    while (*parameter != '\0') {
        bb_data(*parameter);
        parameter = parameter + 1;
    }
    //bb_data(*parameter); //Call bb_data() for each non-NUL character of the string.
}
void bb_display2(const char *parameter  ) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bb_cmd(0xc0); // move the cursor to the lower row (offset 0x40)
    while (*parameter != '\0') {
        bb_data(*parameter);
        parameter = parameter + 1;
    }
}

void setup_bb(void) {
    //configures GPIO Port B for bit-banging the OLED LCD display
    //set pins PB12 (NSS), PB13 (SCK), and PB15 (MOSI)
    //for general purpose output (not an alternate function)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER15);

    GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER15_0;

    //Initialize the ODR so that NSS is high and SCK is low.
    //It does not matter what MOSI is set to.
    GPIOB->ODR &= ~(GPIO_ODR_13);
    GPIOB->ODR |= GPIO_ODR_12;
}


void setup_spi2(void) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //initializes the SPI2 subsystem
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
    GPIOB->MODER |=   GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1; //set these pins to use the alternate functions to do this.
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFR12 | GPIO_AFRH_AFR13 | GPIO_AFRH_AFR15);

//    GPIOB->AFR[1] |= GPIO_AFRH_AFR12 | GPIO_AFRH_AFR13 | GPIO_AFRH_AFR15;

    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR; //Set the baud rate as low as possible (maximum divisor for BR).
                                            //Configure the SPI channel to be in "master mode".
                                                                           //Configure the interface for a 10-bit word size.
    SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_3 | SPI_CR2_DS_0; //Set the SS Output enable bit and enable NSSP.

    SPI2->CR1 |= SPI_CR1_SPE; //Enable the SPI channel.





//Set the baud rate as low as possible (maximum divisor for BR).
//Configure the interface for a 10-bit word size.
//Configure the SPI channel to be in "master mode".
//Set the SS Output enable bit and enable NSSP.
//Enable the SPI channel.
}

void spi_cmd(int parameter) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Waits until the SPI2_SR_TXE bit is set.
    while (!(SPI2->SR & SPI_SR_TXE));
//Copies the parameter to the SPI2_DR.
    SPI2->DR = parameter;
}

void spi_data(int parameter) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Waits until the SPI2_SR_TXE bit is set.
    while (!(SPI2->SR & SPI_SR_TXE));
//Copies the parameter to the SPI2_DR.
    parameter |= 0x200;
    SPI2->DR = (parameter);
}

void spi_init_oled(void) {
    nano_wait(1000000); //Use nano_wait() to wait 1 ms for the display to power up and stabilize.
    spi_cmd(0x38); // set for 8-bit operation
    spi_cmd(0x08); // turn display off
    spi_cmd(0x01); // clear display
    nano_wait(2000000); //Use nano_wait() to wait 2 ms for the display to clear.
    spi_cmd(0x06); // set the display to scroll
    spi_cmd(0x02); // move the cursor to the home position
    spi_cmd(0x0c); // turn the display on
}

void spi_display1(const char *parameter ) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    spi_cmd(0x02); // move the cursor to the home position
    while (*parameter != '\0') {
        spi_data(*parameter);
        parameter = parameter + 1;
    }
 //Call bb_data() for each non-NUL character of the string.
}
void spi_display2(const char *parameter  ) {////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    spi_cmd(0xc0); // move the cursor to the lower row (offset 0x40)
    while (*parameter != '\0') {
        spi_data(*parameter);
        parameter = parameter + 1;
    }
 //Call bb_data() for each non-NUL character of the string.
}

void spi_setup_dma(const short *argument) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; //Enable the RCC clock to the DMA controller

    DMA1_Channel5->CPAR = (uint32_t) & (SPI2->DR); //Set CPAR to the address of the SPI2_DR register.
    DMA1_Channel5->CMAR = argument; //Set CMAR to the parameter passed in.
    DMA1_Channel5->CNDTR = 34; //Set CNDTR to 34.
    DMA1_Channel5->CCR |= DMA_CCR_DIR; //Set the DIRection for copying from-memory-to-peripheral.
    DMA1_Channel5->CCR |= DMA_CCR_MINC; //Set the MINC to increment the CMAR for every transfer.
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0; //Set the memory datum size to 16-bit.
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0; //Set the peripheral datum size to 16-bit.
    DMA1_Channel5->CCR |= DMA_CCR_CIRC; //Set the channel for CIRCular operation.
    SPI2->CR2 |= SPI_CR2_TXDMAEN; //Turn on the configuration bit in SPI2_CR2 that enables a DMA trigger when TX is empty.
}

void enable_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMAEN; //Enable the RCC clock to the DMA controller
    DMA1_Channel5->CCR |= DMA_CCR_EN; //turn on DMA

}

void setup_spi1(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //Enable the RCC clock to GPIO Ports A and B.
    GPIOA->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7);
    GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);

    GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1; //Configure pins PA5 and PA7 for alternate function 0.
    GPIOB->MODER |= GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0; //Configure pins PB10, PB11
    GPIOA->MODER |= GPIO_MODER_MODER3_0; //Configure pins PA3 to be outputs.

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7);
    GPIOB->ODR |= GPIO_ODR_10 | GPIO_ODR_11; //Set the output of each of PB10, PB11 to be '1'.
    GPIOB->ODR |= GPIO_ODR_3; //Set the output of each of PA3 to be '1'.
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //Enable the RCC clock to SPI1.
    //SPI1->CR1 |= ; //Configure SPI1 for Master
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_MSTR | SPI_CR1_SSI;
    SPI1->CR1 &= ~SPI_CR1_BR; //Set the baud rate as low as possible (maximum divisor for BR).
    SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_SPE; //Enable the SPI port.





}

// Write your subroutines above

void show_counter(short buffer[])
{
    for(int i=0; i<10000; i++) {
        char line[17];
        sprintf(line,"% 16d", i);
        for(int b=0; b<16; b++)
            buffer[1+b] = line[b] | 0x200;
    }
}

void internal_clock();
void demo();
void autotest();

extern const Picture *image;

int main(void)
{
    //internal_clock();
    //demo();
    autotest();

    setup_bb();
    bb_init_oled();
    bb_display1("Hello,");
    bb_display2(login);
    setup_spi2();
    spi_init_oled();
    spi_display1("Hello again,");
    spi_display2(login);

    short buffer[34] = {
            0x02, // This word sets the cursor to beginning of line 1.
            // Line 1 consists of spaces (0x20)
            0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220,
            0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220,
            0xc0, // This word sets the cursor to beginning of line 2.
            // Line 2 consists of spaces (0x20)
            0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220,
            0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220, 0x220,
    };

    spi_setup_dma(buffer);
    enable_dma();
    show_counter(buffer);

    setup_spi1();
    LCD_Init(0,0,0);
    LCD_Clear(BLACK);
    LCD_DrawLine(10,20,100,200, WHITE);
    LCD_DrawRectangle(10,20,100,200, GREEN);
    LCD_DrawFillRectangle(120,20,220,200, RED);
    LCD_Circle(50, 260, 50, 1, BLUE);
    LCD_DrawFillTriangle(130,130, 130,200, 190,160, YELLOW);
    LCD_DrawChar(150,155, BLACK, WHITE, 'X', 16, 1);
    LCD_DrawString(140,60,  WHITE, BLACK, "ECE 362", 16, 0);
    LCD_DrawString(140,80,  WHITE, BLACK, "has the", 16, 1);
    LCD_DrawString(130,100, BLACK, GREEN, "best toys", 16, 0);
    LCD_DrawPicture(110,220,(const Picture *)&image);
}
