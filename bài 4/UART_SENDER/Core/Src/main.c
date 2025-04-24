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
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (1 << (6 * 2)); // PA6 output
    GPIOA->MODER &= ~(3 << (0 * 2)); // PA5 input
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PA;
    EXTI->IMR |= EXTI_IMR_IM5;
    EXTI->FTSR |= EXTI_FTSR_TR5;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void delay(volatile uint32_t t) {
    while (t--) __NOP();
}

void USART1_Send(char *str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

volatile uint8_t buttonFlag = 0;
uint8_t ledState = 0;
char rxBuf[20];
volatile uint8_t rxIndex = 0;

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR5) {
        buttonFlag = 1;
        EXTI->PR |= EXTI_PR_PR5;
    }
}

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        char c = USART1->DR;
        rxBuf[rxIndex++] = c;
        if (c == '\n' || rxIndex >= 19) {
            rxBuf[rxIndex] = 0;
            if (strstr(rxBuf, "led on") || strstr(rxBuf, "led off")) {
                for (int i = 0; i < 6; i++) {
                    GPIOA->ODR ^= (1 << 6);
                    delay(800000);
                }
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
        if (buttonFlag) {
            if (ledState == 0) {
                USART1_Send("on led\n");
                ledState = 1;
            } else {
                USART1_Send("off led\n");
                ledState = 0;
            }
            buttonFlag = 0;
        }
    }
}

