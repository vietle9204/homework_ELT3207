#include "stm32f4xx.h"
#include <string.h>

void UART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (2 << (9 * 2)) | (2 << (10 * 2));  // PA9, PA10 alternate function
    GPIOA->AFR[1] |= (7 << (1 * 4)) | (7 << (2 * 4));  // AF7

    USART1->BRR = 0x0683; // 9600 baud @16MHz
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (1 << (13 * 2));
}

void USART1_Send(char *str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

char rxBuf[20];
volatile uint8_t rxIndex = 0;

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        char c = USART1->DR;
        rxBuf[rxIndex++] = c;
        if (c == '\n' || rxIndex >= 19) {
            rxBuf[rxIndex] = 0;
            if (strstr(rxBuf, "on led")) {
                GPIOC->ODR &= ~(1 << 13);
                USART1_Send("led on\n");
            } else if (strstr(rxBuf, "off led")) {
                GPIOC->ODR |= (1 << 13);
                USART1_Send("led off\n");
            }
            rxIndex = 0;
        }
    }
}

int main(void) {
    UART1_Init();
    GPIO_Init();
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 |= USART_CR1_RXNEIE;

    while (1) {
        // Main loop không cần xử lý
    }
}
