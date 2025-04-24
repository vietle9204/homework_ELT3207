#include "stm32f4xx_hal.h"

#define ADC_pin 0
#define PWM_led 8

void init_led();
void init_ADC();
void init_TIM_PWM();
void ADC_IRQHandler(void);
void led_control();

uint16_t adc_value = 0;

int main(void)
{
	init_led();
	init_TIM_PWM();
	init_ADC();
	led_control();
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while(1)
	{

	}
}

void init_led()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //bật clock GPIOA.

	GPIOA->MODER &= ~(3 << (PWM_led * 2));  // Xóa cấu hình cũ

	GPIOA->MODER  |=  (2 << (PWM_led * 2));  // Chọn chế độ Alternate Function (AF)

	GPIOA->AFR[1] |=  (1 << ((PWM_led- 8) * 4)); // Chọn AF1 (TIM1)

}



void init_TIM_PWM()
{

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;       // cấp xung clock cho timer1.

	TIM1->PSC = 99;                           // chia tần: tầ số dếm = 100MHz/(99+1) = 1MH.
	TIM1->ARR = 255;                         //chu kỳ PWM : 0.255ms
	TIM1->CCR1 = 0;		 				  // độ rộng xung mặc định = 0.

	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1->CCMR1 |= 7 << TIM_CCMR1_OC1M_Pos;  //chọn chế độ PWM mode 2.

	TIM1->BDTR |= TIM_BDTR_MOE;			     //cho phép timer1 xuất tín hiệu ngõ ra.
    TIM1->CCER |= TIM_CCER_CC1E;              //chọn kênh CH1 của timer 1(PA8)

	TIM1->CR1 |= TIM_CR1_CEN;                //bật bộ đếm cho timer1.
}

void led_control()
{
	TIM1->CCR1 =  adc_value;    // cập nhật độ rông xung = arr - adc_value : do PWM mode 2
//	TIM1->EGR |= TIM_EGR_UG;           // Cập nhật giá trị mới ngay lập tức
}

void init_ADC()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //bật clock GPIOA.
	GPIOA->MODER &= ~(3 << (ADC_pin * 2));  // Xóa cấu hình cũ
	GPIOA->MODER  |= (3 << (ADC_pin * 2));  // Chọn chế độ analog

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // cấp clock cho ADC1.

	ADC1->CR1 &= ~ADC_CR1_RES_Msk;        //xóa cà đặt cũ
	ADC1->CR1 |= 2 << ADC_CR1_RES_Pos;    //chọn độ phân giải 8 bit;
	ADC1->CR1 |= ADC_CR1_EOCIE;			//bật ngắt cho ADC1.

	ADC1->SMPR2 |= (3 << (ADC_pin * 3));              // Thời gian lấy mẫu 3 cycles

	ADC1->SQR3 |= (ADC_pin << 0);                // Chọn kênh 1 (PA1)

	NVIC_EnableIRQ(ADC_IRQn); 				// cấu hình ngắt chi ADC
	NVIC_SetPriority(ADC_IRQn, 1); 		// Đặt mức ưu tiên cho ngắt ADC

	ADC1->CR2 |= ADC_CR2_CONT;				//chê độ đọc liên tục.
	ADC1->CR2 |= ADC_CR2_ADON;              //bật ADC1.
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & (1 << 1))
    {  // Kiểm tra cờ EOC
        adc_value = ADC1->DR;   	// Đọc giá trị ADC
        led_control();              //cập nhật độ sáng led.
        ADC1->SR &= ~(1 << 1);  	// Xóa cờ EOC
    }
}






