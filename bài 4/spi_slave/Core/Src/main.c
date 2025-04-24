#include "stm32f4xx.h"
#include "string.h"

void SPI1_Slave_Init(void);

#define MAX_BUFFER 16

volatile uint8_t rxBuffer[MAX_BUFFER];  // Bộ đệm nhận dữ liệu từ SPI Master
volatile uint8_t rxIndex = 0;           // Chỉ số ghi dữ liệu vào buffer

int main(void) {
    SPI1_Slave_Init();  // Khởi tạo SPI1 ở chế độ Slave

    // Cấu hình PC13 làm output để điều khiển LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;        // Bật clock GPIOC
    GPIOC->MODER &= ~(3 << (13 * 2));           // Xóa cấu hình cũ
    GPIOC->MODER |= (1 << (13 * 2));            // Đặt PC13 làm output

    NVIC_EnableIRQ(SPI1_IRQn);                  // Kích hoạt ngắt SPI1

    while (1) {
        // Vòng lặp chính không làm gì, xử lý hoàn toàn trong ngắt SPI
    }
}

void SPI1_Slave_Init(void) {
    // Bật clock cho SPI1 và GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;         // SPI1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        // GPIOA clock

    // PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) → cấu hình Alternate Function
    GPIOA->MODER &= ~(0x3 << 2*4 | 0x3 << 2*5 | 0x3 << 2*6 | 0x3 << 2*7); // Xóa mode cũ
    GPIOA->MODER |=  (0x2 << 2*4) | (0x2 << 2*5) | (0x2 << 2*6) | (0x2 << 2*7); // AF mode

    // Gán Alternate Function 5 cho các chân SPI1
    GPIOA->AFR[0] &= ~(0xF << 4*4 | 0xF << 4*5 | 0xF << 4*6 | 0xF << 4*7); // clear trước
    GPIOA->AFR[0] |=  (0x5 << 4*4) | (0x5 << 4*5) | (0x5 << 4*6) | (0x5 << 4*7);

    // ==== CẤU HÌNH SPI1 Ở CHẾ ĐỘ SLAVE ====
    SPI1->CR1 = 0; // Xóa toàn bộ cấu hình trước đó

    // Cấu hình:
    // - Chế độ Slave (MSTR = 0)
    // - Không dùng quản lý phần mềm NSS (SSM = 0)
    // - Dùng mode 0 (CPOL = 0, CPHA = 0)
    SPI1->CR1 &= ~(SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_CPOL | SPI_CR1_CPHA);

    // Cho phép ngắt khi có dữ liệu nhận được (RXNE)
    SPI1->CR2 = SPI_CR2_RXNEIE;

    // Bật SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}

// Trình xử lý ngắt SPI1
void SPI1_IRQHandler(void) {
    if (SPI1->SR & SPI_SR_RXNE) { // Nếu có dữ liệu nhận
        uint8_t data = SPI1->DR;  // Đọc dữ liệu từ thanh ghi DR

        if(data != 0xFF) {  // kiểm tra data không phải dummy
            // Lưu dữ liệu vào buffer nếu còn chỗ
            if (rxIndex < MAX_BUFFER - 1) {
                rxBuffer[rxIndex++] = data;
            }

            // Nếu gặp ký tự kết thúc chuỗi (ví dụ '\n') thì xử lý lệnh
            if (data == '\n') {
                rxBuffer[rxIndex - 1] = '\0'; // Ghi đè '\n' bằng null-terminator

                // So sánh chuỗi lệnh đã nhận
                if (strcmp((char*)rxBuffer, "on led") == 0) {
                    GPIOC->ODR |= (1 << 13);  // Bật LED PC13

                    // Gửi phản hồi  trạng thái led 0x01 cho Master
                    while (!(SPI1->SR & SPI_SR_TXE)); // Chờ TXE
                    SPI1->DR = 0x01;
                } else if (strcmp((char*)rxBuffer, "off led") == 0) {
                    GPIOC->ODR &= ~(1 << 13); // Tắt LED PC13

                    // Gửi phản hồi trạng thái led 0x00 cho Master
                    while (!(SPI1->SR & SPI_SR_TXE));
                    SPI1->DR = 0x00;
                }

                rxIndex = 0; // Reset buffer sau khi xử lý xong lệnh
            }
        }
        else {
            // Nếu là dữ liệu rác (ví dụ 0xFF , đây là byte master gửi để tạo sck, slave gửi byte đã đc nạp vào DR trước đó), đọc bỏ
            (void)SPI1->DR;
        }
    }
}
