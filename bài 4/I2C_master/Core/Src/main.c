#include "stm32f4xx.h"
#include <string.h>

#define SLAVE_ADDR  0x27  // Địa chỉ Slave (7-bit)

void I2C1_Init(void);
void I2C1_Write(uint8_t addr, uint8_t *data, uint8_t len);
void I2C1_Read(uint8_t addr, uint8_t *data, uint8_t len);
void delay_ms(uint32_t ms);
void LED_Blink(void);

int main(void) {
    // 1. Cấu hình PA5 (Button) và PA6 (LED)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODE5); // PA5: Input
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_0; // Pull-up
    GPIOA->MODER |= GPIO_MODER_MODE6_0; // PA6: Output

    // 2. Cấu hình I2C1
    I2C1_Init();

    uint8_t led_on_msg[] = "led on";
    uint8_t led_off_msg[] = "led off";
    uint8_t response[7];

    while (1) {
        if (!(GPIOA->IDR & GPIO_IDR_ID5)) {  // Nút nhấn PA5 được nhấn
            delay_ms(50); // Chống dội phím
            while (!(GPIOA->IDR & GPIO_IDR_ID5)); // Chờ thả nút

            I2C1_Write(SLAVE_ADDR, led_on_msg, strlen((char*)led_on_msg)); // Gửi "led on"
            delay_ms(100);
            I2C1_Read(SLAVE_ADDR, response, 6); // Nhận phản hồi
            response[6] = '\0'; // Thêm kết thúc chuỗi

            if (strcmp((char*)response, "led on") == 0) {
                LED_Blink(); // Nháy LED PA6
            } else if (strcmp((char*)response, "led off") == 0) {
                LED_Blink();
            }
        }
    }
}

void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4));
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;

    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = 42; // APB1 = 42MHz
    I2C1->CCR = 210; // 100kHz
    I2C1->TRISE = 43;
}

void I2C1_Write(uint8_t addr, uint8_t *data, uint8_t len) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (addr << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR2;

    for (uint8_t i = 0; i < len; i++) {
        I2C1->DR = data[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_Read(uint8_t addr, uint8_t *data, uint8_t len) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (addr << 1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR2;

    for (uint8_t i = 0; i < len - 1; i++) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        data[i] = I2C1->DR;
    }

    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    data[len - 1] = I2C1->DR;
}

void LED_Blink(void) {
    for (int i = 0; i < 3; i++) {
        GPIOA->ODR ^= GPIO_ODR_OD6;
        delay_ms(500);
    }
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();
    }
}
