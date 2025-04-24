#include "stm32f4xx_hal.h"

#define motor_ena  8  // Chân điều khiển PWM
#define motor_in1  9  // Chân điều khiển chiều quay
#define motor_in2  10 // Chân điều khiển chiều quay
#define ADC_pin    0  // Chân đọc giá trị chiết áp
#define button1    12 // Chân nút nhấn đảo chiều motor

#define MAX_SPEED  4095  // Giới hạn tốc độ tối đa (12-bit)
#define MIN_SPEED  0     // Giới hạn tốc độ tối thiểu

uint16_t motor_state_speed = 0; // Tốc độ động cơ
uint8_t motor_state_direction = 0; // Hướng quay động cơ

void init_motor_DC();
void init_TIM_PWM();
void init_ADC();
void motor_speed();
void motor_direction();
void init_interrupt();
void init_button();

int main(void)
{
    init_motor_DC();
    init_TIM_PWM();
    init_ADC();
    init_interrupt();
    init_button();
    motor_direction();
    motor_speed();

    // Bắt đầu chuyển đổi ADC
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while(1)
    {
        // Vòng lặp chính - chờ xử lý ngắt
    }
}

/**
 * Khởi tạo GPIO điều khiển động cơ
 */
void init_motor_DC()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA

    // Cấu hình PA8 (PWM - TIM1_CH1)
    GPIOA->MODER  |= (2 << (motor_ena * 2));
    GPIOA->AFR[1] |= (1 << ((motor_ena - 8) * 4)); // AF1 - TIM1_CH1

    // Cấu hình PA9 (IN1) & PA10 (IN2) làm output
    GPIOA->MODER  |= (1 << (motor_in1 * 2)) | (1 << (motor_in2 * 2));
    GPIOA->PUPDR  &= ~((3 << (motor_in1 * 2)) | (3 << (motor_in2 * 2))); // Không kéo lên/xuống
}

/**
 * Khởi tạo Timer1 để tạo PWM
 */
void init_TIM_PWM()
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Bật clock TIM1

    TIM1->PSC = 10 - 1;  // Tần số đếm = 100MHz / 10 = 10MHz
    TIM1->ARR = MAX_SPEED; // Giá trị PWM theo độ phân giải ADC (12-bit)
    TIM1->CCR1 = MAX_SPEED / 2; // Mặc định 50%

    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM Mode 1
    TIM1->BDTR  |= TIM_BDTR_MOE; // Cho phép xuất PWM
    TIM1->CCER  |= TIM_CCER_CC1E; // Kích hoạt kênh CH1 (PA8)

    TIM1->CR1   |= TIM_CR1_CEN; // Bắt đầu timer
}

/**
 * Khởi tạo ADC để đọc chiết áp điều khiển động cơ
 */
void init_ADC()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA
    GPIOA->MODER |= (3 << (ADC_pin * 2)); // Chế độ Analog

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Bật clock ADC1

    ADC1->CR1 &= ~ADC_CR1_RES_Msk;
    ADC1->CR1 |= (0 << ADC_CR1_RES_Pos); // Độ phân giải 12-bit

    ADC1->CR1 |= ADC_CR1_EOCIE; // Bật ngắt ADC
    ADC1->SQR3 = ADC_pin; // Kênh ADC0
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON; // Chế độ liên tục & bật ADC

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1);
}

/**
 * Cập nhật tốc độ động cơ
 */
void motor_speed()
{
    if (motor_state_speed > MAX_SPEED) motor_state_speed = MAX_SPEED;
    if (motor_state_speed < MIN_SPEED) motor_state_speed = MIN_SPEED;

    TIM1->CCR1 = motor_state_speed; // Cập nhật giá trị PWM
}

/**
 * Cập nhật hướng quay động cơ
 */
void motor_direction()
{
    if (motor_state_direction == 0)
    {
        GPIOA->BSRR = (1 << motor_in1); // Bật IN1
        GPIOA->BSRR = (1 << (16 + motor_in2)); // Tắt IN2
    }
    else
    {
        GPIOA->BSRR = (1 << motor_in2); // Bật IN2
        GPIOA->BSRR = (1 << (16 + motor_in1)); // Tắt IN1
    }
}

/**
 * Xử lý ngắt ADC
 */
void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_EOC)
    {
        motor_state_speed = ADC1->DR; // Đọc giá trị ADC
        motor_speed();
        ADC1->SR &= ~ADC_SR_EOC; // Xóa cờ EOC
    }
}

/**
 * Khởi tạo nút nhấn
 */
void init_button()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Bật clock GPIOB
    GPIOB->MODER &= ~(3 << (button1 * 2)); // PB12 - Input mode
}

/**
 * Khởi tạo ngắt ngoài cho nút nhấn
 */
void init_interrupt()
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Bật clock SYSCFG

    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB; // Kết nối PB12 với EXTI12
    EXTI->IMR  |= EXTI_IMR_IM12; // Cho phép ngắt EXTI12
    EXTI->FTSR |= EXTI_FTSR_TR12; // Ngắt cạnh xuống

    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * Xử lý ngắt ngoài từ nút nhấn
 */
void EXTI15_10_IRQHandler()
{
    if (EXTI->PR & (1 << 12))
    {
        motor_state_direction = (motor_state_direction == 0) ? 1 : 0;
        motor_direction();

        EXTI->PR |= (1 << 12); // Xóa cờ ngắt
    }
}

