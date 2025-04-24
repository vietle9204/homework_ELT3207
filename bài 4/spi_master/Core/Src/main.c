#include "stm32f4xx.h"
#include "string.h"

// Khai báo hàm
void Button_Init(void);
void SPI1_Init(void);
void BlinkLED_PA3(uint8_t times);
void SPI_Transmit(uint8_t *data, uint8_t size);
void delay_ms(uint32_t ms);

// Biến toàn cục lưu trạng thái LED: 0 = tắt, 1 = bật
uint8_t ledState = 0;

// Bộ đệm nhận dữ liệu SPI
#define BUFFER_SIZE 16
volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_index = 0;

int main(void) {
    SPI1_Init();          // Khởi tạo SPI1 (Master)
    Button_Init();        // Khởi tạo nút nhấn trên PA2

    // Cấu hình chân PA3 làm output (LED)
    GPIOA->MODER &= ~(0x3 << (3 * 2));  // Xóa 2 bit MODER3
    GPIOA->MODER |=  (0x1 << (3 * 2));  // Thiết lập PA3 là output

    // Kích hoạt ngắt EXTI2 và SPI1
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);

    // Nhấp nháy LED 3 lần khi khởi động
    BlinkLED_PA3(3);

    while (1) {
        // Vòng lặp chính không làm gì, chỉ chờ ngắt xử lý
    }
}

// Hàm cấu hình nút nhấn tại PA2 (dùng EXTI2)
void Button_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Bật clock GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Bật clock SYSCFG (để cấu hình EXTI)

    GPIOA->MODER &= ~(3 << (2 * 2));         // PA2 là input (00)

    SYSCFG->EXTICR[0] &= ~(0xF << (2 * 4));  // Chọn PA2 làm nguồn EXTI2

    EXTI->IMR  |= (1 << 2);                  // Cho phép mask interrupt EXTI2
    EXTI->FTSR |= (1 << 2);                  // Ngắt khi sườn xuống (nút nhấn)
}

// Hàm khởi tạo SPI1 ở chế độ Master
void SPI1_Init(void) {
    // Bật clock SPI1 và GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA4-PA7 làm chức năng thay thế (AF5 - SPI1)
    GPIOA->MODER &= ~(0x3 << 2*4 | 0x3 << 2*5 | 0x3 << 2*6 | 0x3 << 2*7);
    GPIOA->MODER |=  (0x2 << 2*4 | 0x2 << 2*5 | 0x2 << 2*6 | 0x2 << 2*7);

    // Gán AF5 cho các chân PA4–PA7
    GPIOA->AFR[0] &= ~((0xF << 4*4) | (0xF << 4*5) | (0xF << 4*6) | (0xF << 4*7));
    GPIOA->AFR[0] |=  (0x5 << 4*4 | 0x5 << 4*5 | 0x5 << 4*6 | 0x5 << 4*7);

    // Reset các thiết lập SPI1
    SPI1->CR1 = 0;

    SPI1->CR1 |= SPI_CR1_MSTR;              // Master mode
    SPI1->CR1 &= ~SPI_CR1_SSM;              // Tắt chế độ quản lý NSS bằng phần mềm
    SPI1->CR2 |= SPI_CR2_SSOE;              // Cho phép xuất NSS bằng phần cứng
    SPI1->CR2 |= SPI_CR2_RXNEIE;            // Kích hoạt ngắt khi RXNE = 1

    SPI1->CR1 &= ~(SPI_CR1_BR);             // Xóa bits tốc độ Baud Rate
    SPI1->CR1 |= (0x1 << SPI_CR1_BR_Pos);   // Tốc độ SPI = fPCLK / 4
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // SPI mode 0 (CPOL=0, CPHA=0)
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;         // Truyền bit cao trước (MSB first)

    SPI1->CR1 |= SPI_CR1_SPE;               // Enable SPI
}

// Hàm truyền dữ liệu qua SPI (Master gửi từng byte)
void SPI_Transmit(uint8_t *data, uint8_t size) {
    for (int i = 0; i < size; i++) {
        SPI1->DR = data[i];                    // Gửi byte vào thanh ghi dữ liệu
        while (!(SPI1->SR & SPI_SR_TXE));      // Chờ cho đến khi TXE = 1 (truyền xong)
        while (SPI1->SR & SPI_SR_BSY);         // Chờ SPI không bận
        (void)SPI1->DR;                         // Đọc DR để xóa cờ (khi không cần đọc data)
    }

    // Gửi thêm 2 byte dummy : 0xFF để tạo SCK cho slave phản hồi.
    delay_ms(100);
    SPI1->DR = 0xFF;
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = 0xFF;
        while (!(SPI1->SR & SPI_SR_TXE));
}

// Hàm xử lý ngắt từ EXTI2 (nút nhấn)
void EXTI2_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR2) {           // Kiểm tra cờ ngắt EXTI2
        EXTI->PR = EXTI_PR_PR2;             // Xóa cờ ngắt bằng cách ghi 1

        // Gửi dữ liệu SPI tùy theo trạng thái hiện tại
        if (ledState == 0) {
            SPI_Transmit((uint8_t *)"on led\n", 7);   // Gửi chuỗi "on led"
        } else {
            SPI_Transmit((uint8_t *)"off led\n", 8);  // Gửi chuỗi "off led"
        }
    }
}

// Hàm xử lý ngắt SPI1 (nhận dữ liệu)
void SPI1_IRQHandler(void) {
    if (SPI1->SR & SPI_SR_RXNE) {       // Nếu có dữ liệu trong DR
        uint8_t byte = SPI1->DR;        // Đọc byte nhận được

        // Kiểm tra dữ liệu nhận
        if (byte == 0x01) {
            ledState = 1;               // Đặt trạng thái LED là bật
            BlinkLED_PA3(3);            // Nhấp nháy LED 3 lần
        } else if (byte == 0x00) {
            ledState = 0;               // Đặt trạng thái LED là tắt
            BlinkLED_PA3(5);            // Nhấp nháy LED 5 lần
        }
    }
}

// Hàm nhấp nháy LED tại chân PA3 `times` lần
void BlinkLED_PA3(uint8_t times) {
    for (int i = 0; i < times; i++) {
        GPIOA->ODR |= (1 << 3);        // Bật LED tại PA3
        delay_ms(500);                // Delay 500ms
        GPIOA->ODR &= ~(1 << 3);       // Tắt LED
        delay_ms(500);
    }
}

// Hàm delay (giả lập delay theo CPU cycles)
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP();  // Lệnh không làm gì, chỉ để tạo độ trễ
    }
}
