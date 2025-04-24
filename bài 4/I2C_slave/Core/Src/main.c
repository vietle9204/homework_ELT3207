#include "stm32f4xx.h"
#include <string.h>

void I2C1_Init(void);
void I2C1_Write(uint8_t *data, uint8_t len);
void I2C1_Read(uint8_t *data, uint8_t len);

uint8_t buffer[7];

int main(void) {
    // Cấu hình PC13 (LED)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODE13_0;

    // Cấu hình I2C1
    I2C1_Init();

    while (1) {
        I2C1_Read(buffer, 6);
        buffer[6] = '\0';

        if (strcmp((char*)buffer, "led on") == 0) {
            GPIOC->BSRR = GPIO_BSRR_BS13;
            I2C1_Write((uint8_t*)"led on", 6);
        } else if (strcmp((char*)buffer, "led off") == 0) {
            GPIOC->BSRR = GPIO_BSRR_BR13;
            I2C1_Write((uint8_t*)"led off", 6);
        }
    }
}

void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->OAR1 = (0x27 << 1);
    I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
}

void I2C1_Write(uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->SR1 & I2C_SR1_TXE));
        I2C1->DR = data[i];
    }
}

void I2C1_Read(uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        data[i] = I2C1->DR;
    }
}
