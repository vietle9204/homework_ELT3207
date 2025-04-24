#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "ssd1306.h"


I2C_HandleTypeDef hi2c1;

int scope_state = 0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void ADC_Init(void);
static void MX_I2C1_Init(void);


void init_button(void)
{


	    // PA5: input (00), không cần chỉnh vì mặc định là input
	    GPIOA->MODER &= ~(3 << (5 * 2));

	{
	    // Bật clock cho SYSCFG
	    RCC->APB2ENR |= (1 << 14);

	    // Kết nối EXTI5 với PA5 (EXTI5 -> PORTA)
	    SYSCFG->EXTICR[1] &= ~(0xF << 4);  // EXTI5 nằm ở EXTICR[1], bit 7:4

	    // Bật mask ngắt cho EXTI5
	    EXTI->IMR |= (1 << 5);

	    // Chọn cạnh lên (RISING)
	    EXTI->FTSR |= 1 << 5;  // Không dùng cạnh xuống

	    // Bật ngắt EXTI9_5 trong NVIC
	    NVIC_EnableIRQ(EXTI9_5_IRQn);
	    NVIC_SetPriority(EXTI9_5_IRQn, 1);
	}

}

// Hàm xử lý ngắt EXTI Line 5–9
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1 << 5))  // Kiểm tra có ngắt tại EXTI5 không
    {
    	// Xóa cờ ngắt
    	        EXTI->PR |= (1 << 5);

    	scope_state ^= 1;

    }
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  ADC_Init();
  MX_I2C1_Init();
  init_button();

  ssd1306_init(&hi2c1);

  ssd1306_clear();

  char *str = "HELLO";
  for(int i = 0; i < 5; i++)
  	    {
  	           ssd1306_draw_char(i*6, 0, str[i]);
  	    }

  while (1)
  {
    /* USER CODE END WHILE */
      if(scope_state == 1)
      {
	    ADC1->CR2 |= ADC_CR2_SWSTART;
      }
    /* USER CODE BEGIN 3 */
  }
}


static void ADC_Init(void)
{
// Cấu hình GPIOA chế độ analog để đọc ADC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  	// Bật clock GPIOA
	GPIOA->MODER &= ~(3 << (0 * 2));  		// Xóa cấu hình cũ của PA0
	GPIOA->MODER  |= (3 << (0 * 2));  		// Chọn chế độ analog cho PA0 (MODER00 = 11)

	// Cấu hình bộ ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Bật clock cho ADC1

	ADC1->CR1 &= ~ADC_CR1_RES_Msk;        	// Xóa cấu hình độ phân giải cũ (để defaut 12bit)
	ADC1->CR1 |= ADC_CR1_EOCIE;				// Bật ngắt khi ADC chuyển đổi hoàn tất (EOC)

	ADC1->SMPR2 |= (3 << (0 * 3));          // Cấu hình thời gian lấy mẫu cho kênh 0: 56 cycles

	ADC1->SQR3 &= ~(0xF << 0);              // Chọn kênh ADC = kênh 0 (PA0)

	ADC1->CR2 |= ADC_CR2_ADON;            	// Bật ADC1 (Enable ADC)

	NVIC_EnableIRQ(ADC_IRQn); 				// Bật ngắt ADC trong NVIC
	NVIC_SetPriority(ADC_IRQn, 1); 			// Ưu tiên mức 1 cho ngắt ADC
}

void USB_Send_String(char *msg)
{
    uint8_t len = strlen(msg);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

float Rs_caculator(uint16_t ADC_value)
{
	return (float)(10.0*(4095 - ADC_value)/ADC_value);
}

void ADC_IRQHandler(void)
{
	uint16_t adc_value;
	float Rs_value;
	char buffer[16];
	char str_adc[12];
	if (ADC1->SR & (1 << 1)) 	// Kiểm tra cờ EOC (End Of Conversion)
	{
	    adc_value = ADC1->DR;   			// Đọc giá trị ADC
	    Rs_value = Rs_caculator(adc_value); 	// Tính toán ppm từ giá trị ADC


	    sprintf(buffer, "%8f\n", Rs_value);
	    sprintf(str_adc, "%d\n", adc_value);
	    // gửi fs
	    USB_Send_String(buffer);
	    USB_Send_String(str_adc);
	    // gửi I@C

	    char *str = "Rs                  ";
	    for(int i = 0; i < 16; i++)
	    {
	           ssd1306_draw_char(i*6, 0, str[i]);
	    }

	    for(int i = 0; i < 16; i++)
	    {
	           ssd1306_draw_char(i*6, 1, buffer[i]);
	    }

	    str = "ADC              ";
	    for(int i = 0; i < 16; i++)
	    {
	           ssd1306_draw_char(i*6, 3, str[i]);
	    }

	    for(int i = 0; i < 16; i++)
	    {
	         ssd1306_draw_char(i*6, 4, str_adc[i]);
	    }

	    ADC1->SR &= ~(1 << 1);  			// Xóa cờ EOC
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
	// 1. Bật clock GPIOB
		    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

		    // 2. Bật clock I2C1
		    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

		    // 3. Cấu hình PB6 và PB7 chế độ alternate function (AF)
		    GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);  // Clear bits
		    GPIOB->MODER |= (0b10 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE7_Pos); // Alternate function

		    // 4. Chọn AF4 cho I2C1
		    GPIOB->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL6_Pos) | (0xF << GPIO_AFRL_AFSEL7_Pos)); // Clear
		    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos);        // AF4

		    // 5. Output type open-drain
		    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

		    // 6. Pull-up
		    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk);
		    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD6_Pos) | (0b01 << GPIO_PUPDR_PUPD7_Pos);
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
