#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

#define ADC1_DR_ADDRESS  ((uint32_t)0x4001204C) // Địa chỉ thanh ghi dữ liệu ADC1

uint8_t adc_value_8bit;
char msg[20];
uint32_t read_delay_ms = 200; // Thời gian chờ giữa các lần đọc ADC

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USB_DEVICE_Init(void);
void ADC1_Init(void);
uint8_t ADC1_Read(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    ADC1_Init();  // Cấu hình ADC 8-bit

    while (1) {
        adc_value_8bit = ADC1_Read();  // Đọc ADC 8-bit

        // Gửi giá trị ADC qua USB CDC
        int len = sprintf(msg, "%d\r\n", adc_value_8bit);
        CDC_Transmit_FS((uint8_t*)msg, len);

        HAL_Delay(read_delay_ms); // Điều chỉnh tốc độ gửi dữ liệu
    }
}

// ====== Cấu hình ADC 8-bit ======
void ADC1_Init(void) {
    // Bật clock ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Cấu hình độ phân giải ADC = 8-bit (RES[1:0] = 10)
    ADC1->CR1 &= ~ADC_CR1_RES;  // Xóa các bit RES trước
    ADC1->CR1 |= (0b10 << ADC_CR1_RES_Pos);  // Đặt độ phân giải 8-bit

    // Cấu hình kênh ADC1 cho PA0 (Channel 1)
    ADC1->SQR3 = 0;  // Chọn kênh 0 (PA0)
    ADC1->SQR1 = 0;  // Chỉ một lần chuyển đổi (1 conversion)

    // Thiết lập thời gian lấy mẫu (Sampling time)
    ADC1->SMPR2 |= (0b011 << ADC_SMPR2_SMP1_Pos);  // 56 chu kỳ (giảm nhiễu)

    // Cấu hình ADC ở chế độ một lần chuyển đổi (Single Conversion)
    ADC1->CR1 &= ~ADC_CR1_SCAN;  // Không dùng scan mode
    ADC1->CR2 |= ADC_CR2_ADON;  // Bật ADC
}

// ====== Đọc ADC 8-bit ======
uint8_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;  // Bắt đầu chuyển đổi
    while (!(ADC1->SR & ADC_SR_EOC));  // Chờ hoàn thành
    return (uint8_t) ADC1->DR;  // Đọc giá trị ADC 8-bit
}

// ====== Cấu hình clock hệ thống ======
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 15;
    RCC_OscInitStruct.PLL.PLLN = 144;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        while(1);
    }
}

// ====== Cấu hình GPIO ======
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Bật clock GPIOA
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // Vòng lặp vô hạn để dừng chương trình khi có lỗi
  }
}
