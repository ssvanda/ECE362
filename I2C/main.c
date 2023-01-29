
//===========================================================================
// ECE 362 lab experiment 9 -- I2C
//===========================================================================

#include "stm32f0xx.h"
#include <stdint.h> // for uint8_t
#include <string.h> // for strlen() and strcmp()

// Be sure to change this to your login...
const char login[] = "ssvanda";

//================================= ==========================================
// Wait for n nanoseconds. (Maximum time: 4.294 seconds)
//===========================================================================
void nano_wait(unsigned int n) {
    asm(    "         mov r0,%0\n"
            "repeat:  sub r0,#83\n"
            "         bgt repeat\n" : : "r"(n) : "r0", "cc");
}

// Write your subroutines below...
void setup_i2c(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->AFR[0] &= ~(GPIO_AFRH_AFR8 | GPIO_AFRH_AFR9);
    GPIOB->AFR[1] |=  1 << (4*0) | 1 << (4*1);

//I2C CR1 CONFIG
    I2C1->CR1 &= ~(I2C_CR1_PE); //disable the PE bit in CR1 before making the following configuration changes.
    I2C1->CR1 &= ~(I2C_CR1_ANFOFF); //Turn off the ANFOFF bit (turn on the analog noise filter).
    I2C1->CR1 &= ~(I2C_CR1_ERRIE); //Disable the error interrupt.
    I2C1->CR1 &= ~(I2C_CR1_NOSTRETCH); //Turn off the NOSTRETCH bit in CR1 to enable clock stretching.
//From table 83 p 642 of FRM. Set for 400 kHz with 8 MHz clock
    I2C1->TIMINGR = 0;
    I2C1->TIMINGR &= ~(I2C_TIMINGR_PRESC); //clear prescaler
    I2C1->TIMINGR |= 0 << 28; //(I2C_TIMINGR_PRESC); Set the prescaler to 0.
    I2C1->TIMINGR |= 3 << 20; //(I2C_TIMINGR_SCLDEL); Set the SCLDEL field to 3.
    I2C1->TIMINGR |= 1 << 16; //(I2C_TIMINGR_SDADEL); Set the SDADEL field to 1.
    I2C1->TIMINGR |= 3 << 8;  //(I2C_TIMINGR_SCLH); Set the SCLH field to 3.
    I2C1->TIMINGR |= 9 << 0;  //(I2C_TIMINGR_SCLL); Set the SCLL field to 9.

    I2C1->OAR1 &= ~(I2C_OAR1_OA1EN); //Disable both of the "own addresses", OAR1 and OAR2.
    I2C1->OAR2 &= ~(I2C_OAR2_OA2EN); //Disable both of the "own addresses", OAR1 and OAR2.

    I2C1->CR2 &= ~(I2C_CR2_ADD10); //Configure the ADD10 field of CR2 for 7-bit mode. ///0 = 7 bit mode, 1 = 10 bit
    I2C1->CR2 |= (I2C_CR2_AUTOEND); //Turn on the AUTOEND setting to enable automatic end.
    I2C1->CR1 |= (I2C_CR1_PE); //Enable the channel by setting the PE bit in CR1.



}
void i2c_start(uint32_t devaddr, uint8_t size, uint8_t dir) {
 // dir: 0 = master requests a write transfer
 // dir: 1 = master requests a read transfer
    uint32_t tmpreg = I2C1->CR2;
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
    if (dir == 1)
        tmpreg |= I2C_CR2_RD_WRN; // Read from slave
    else
        tmpreg &= ~I2C_CR2_RD_WRN; // Write to slave
        tmpreg |= ((devaddr<<1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);
        tmpreg |= I2C_CR2_START;
        I2C1->CR2 = tmpreg;
}
void i2c_stop(void) {
    if (I2C1->ISR & I2C_ISR_STOPF)
        return;
 // Master: Generate STOP bit after current byte has been transferred.
    I2C1->CR2 |= I2C_CR2_STOP;
 // Wait until STOPF flag is reset
    while(!(I2C1->ISR & I2C_ISR_STOPF));
        I2C1->ICR |= I2C_ICR_STOPCF; // Write to clear STOPF flag
}
void i2c_waitidle(void) {
    while ( (I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY); // while busy, wait.
}

int8_t i2c_senddata(uint8_t devaddr, void *pdata, uint8_t size) {
    int i;
    if (size <= 0 || pdata == 0)
        return -1;
    uint8_t *udata = (uint8_t*)pdata;
    i2c_waitidle();
    i2c_start(devaddr, size, 0); // Last argument is dir: 0 = sending data to the slave.
    for(i=0; i<size; i++) {
 // TXIS bit is set by hardware when the TXDR register is empty and the
 // data to be transmitted must be written in the TXDR register. It is
 // cleared when the next data to be sent is written in the TXDR reg

 // The TXIS flag is not set when a NACK is received.
        int count = 0;
        while( (I2C1->ISR & I2C_ISR_TXIS) == 0) {
            count += 1;
            if (count > 1000000)
                return -1;
            if (i2c_checknack()) {
                i2c_clearnack();
                i2c_stop();
                return -1;
         }

     }
 // TXIS is cleared by writing to the TXDR register.
     I2C1->TXDR = udata[i] & I2C_TXDR_TXDATA;
}
 // Wait until TC flag is set or the NACK flag is set.
    while((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);
    if ( (I2C1->ISR & I2C_ISR_NACKF) != 0)
        return -1;
    i2c_stop();
 return 0;
}

int8_t i2c_recvdata(uint8_t SlaveAddress, uint8_t *pData, uint8_t Size) {
    if (Size <= 0 || pData == NULL)
        return -1;
    i2c_waitidle();
    i2c_start(SlaveAddress, Size, 1);   // 1 = Receiving from the slave
    for (int i = 0; i < Size; i++) {  // Wait until RXNE flag is set
        while( (I2C1->ISR & I2C_ISR_RXNE) == 0);
        pData[i] = I2C1->RXDR & I2C_RXDR_RXDATA;
     }
     while((I2C1->ISR & I2C_ISR_TC) == 0); // Wait until TCR flag is set
     i2c_stop();
     return 0;
}


void i2c_set_iodir(uint8_t parameter) { // accepts a single integer parameter between 0 and 255
    //writes it into the MCP23008 IODIR register via the I2C interface
//address of the MCP23008: 0100 111
//address of IODIR: 00h

    // dir: 0 = master requests a write transfer
    // dir: 1 = master requests a read transfer
    uint8_t devaddr = 0x27;
    uint8_t size = 2;
    uint8_t addr = 0;
    char data[2] = {addr, parameter};
    i2c_senddata(devaddr, data, size);
}
void i2c_set_gpio(uint8_t parameter) {
   //address of the MCP23008: 0100110
   //address of GPIO: 09h
    // dir: 0 = master requests a write transfer
    // dir: 1 = master requests a read transfer
    uint8_t devaddr = 0x27;
    uint8_t size = 2;
    uint8_t addr = 9;

    char data[2] = {addr, parameter};
    i2c_senddata(devaddr, data, size);
}

uint8_t i2c_get_gpio() {
    //address of the MCP23008: 0100
    //address of GPIO: 09h
    // dir: 0 = master requests a write transfer
    // dir: 1 = master requests a read transfer
    uint8_t devaddr = 0x27; //0b 0100 111
    uint8_t size = 1;
    uint8_t addr = 9;
    //uint8_t parameter;
    char data[1] = {addr};

    i2c_senddata(devaddr, data, size);
    i2c_recvdata(devaddr, data, size);                 //I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t Size
    return data[0];
}

void i2c_write_flash(uint16_t loc, const char *data, uint8_t len) {
    //write len bytes at the address data into the EEPROM at the storage location loc
    //create a 34-byte buffer
    //set the first two bytes to the 12-bit storage location
    //following lenbytes should be copied from data
    //loc = 0xabc
    uint8_t EEPROM = 0x57;
    uint8_t buffer[34];
    buffer[0] = loc >> (2*4);
    buffer[1] = (loc & 0xff);
    for(int i = 2; i < (len + 2); i++) {
        buffer[i] = data[i - 2];
    }

    i2c_senddata(EEPROM, buffer, (len + 2));


}

int i2c_write_flash_complete() {
    uint8_t EEPROM = 0x57;

    //Wait for the I2C channel to be idle.
    i2c_waitidle();
    //Initiate an i2c_start() with the correct I2C EEPROM device ID, zero length, and write-intent.
    i2c_start(EEPROM, 0, 0);
    //Wait until either the TC flag or NACKF flag is set.
    while((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);
    //If the NACKF flag is set, clear it, invoke i2c_stop() and return 0.
    if (i2c_checknack()) {
        i2c_clearnack();
        i2c_stop();
        return 0;
    }
    //If the NACKF flag is not set, invoke i2c_stop() and return 1.
    if (i2c_checknack() == 0) {
        i2c_stop();
        return 1;
    }
}
void i2c_read_flash(uint16_t loc, char data[], uint8_t len) {
    //read len bytes into the array data from the EEPROM at the storage location loc
    //do a two-byte write to set the location to read from followed by a read of the requested length.
    uint8_t EEPROM = 0x57;
    char buffer[2] = {(loc & 0xf00) >> 8, loc & 0xff};

    i2c_senddata(EEPROM, buffer, 2);
    i2c_recvdata(EEPROM, data, len);

}
int i2c_checknack(void) {
    if(I2C1->ISR & I2C_ISR_NACKF)
        return 1;
    else
        return 0;
}
void i2c_clearnack(void) {
    I2C1->ISR |= I2C_ISR_NACKF;
}

void internal_clock();
void demo();
void autotest();

int main(void)
{
    //internal_clock();
    //demo();
    autotest();

    setup_i2c();
    //i2c_test();

    i2c_set_iodir(0xf0); //  upper 4 bits input / lower 4 bits output

    // Show happy LEDs for 4 seconds.
    for(int i=0; i<10; i++) {
        for(int n=1; n <= 8; n <<= 1) {
            i2c_set_gpio(n);
            int value = i2c_get_gpio();
            if ((value & 0xf) != n)
                break;
            nano_wait(100000000); // 0.1 s
        }
    }

    const char string[] = "This is a test.";
    int len = strlen(string) + 1;
    i2c_write_flash(0x200, string, len);

    int count = 0;
    while(1) {
        if (i2c_write_flash_complete())
            break;
        count++;
    }

    if (count == 0) {
        // That could not have completed immediately.
        // i2c_write_flash_complete() does not work.  Show the slow angry LEDs.
        int all = 0xf;
        for(;;) {
            i2c_set_gpio(all);
            all ^= 0xf;
            nano_wait(500000000);
        }
    }

    char readback[100];
    i2c_read_flash(0x200, readback, len);
    if (strcmp(string,readback) == 0) {
        // The string comparison matched.  Show happy LEDs.
        for(;;) {
            for(int n=1; n <= 8; n <<= 1) {
                i2c_set_gpio(n);
                int value = i2c_get_gpio();
                if ((value & 0xf) != n)
                    break;
                nano_wait(100000000); // 0.1 s
            }
        }
    } else {
        // The string comparison failed.  Show angry LEDs.
        int all = 0xf;
        for(;;) {
            i2c_set_gpio(all);
            all ^= 0xf;
            nano_wait(100000000);
        }
    }
}
