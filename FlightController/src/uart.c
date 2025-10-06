#include "uart.h"
#include "stm32f4xx.h"

// BLE module baud rate - change if your module uses different rate
#define BLE_BAUD_RATE 115200 // hc-05

void uart_init(void) {
  // Enable GPIOA and USART2 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure PA2 (USART2_TX) and PA3 (USART2_RX) as alternate function
  // Clear mode bits first
  GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
  // Set to alternate function mode (10)
  GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));

  // Set alternate function to AF7 (USART2) for PA2 and PA3
  GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
  GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));

  // Set output speed to high for better signal integrity
  GPIOA->OSPEEDR |= (3 << (2 * 2)) | (3 << (3 * 2));

  // Configure pull-up on RX pin for noise immunity
  GPIOA->PUPDR &= ~(3 << (3 * 2));
  GPIOA->PUPDR |= (1 << (3 * 2)); // Pull-up on PA3 (RX)

  // Calculate baud rate register value
  // APB1 clock is 50MHz (100MHz system clock / 2)
  // BRR = APB1_Clock / Baud_Rate
  uint32_t apb1_clock = 50000000;
  USART2->BRR = apb1_clock / BLE_BAUD_RATE;

  // Configure USART2:
  // - 8 data bits (default)
  // - 1 stop bit (default)
  // - No parity (default)
  // - Enable transmitter and receiver
  USART2->CR1 = USART_CR1_TE | // Transmitter enable
                USART_CR1_RE | // Receiver enable
                USART_CR1_UE;  // USART enable
}

void uart_send_byte(uint8_t data) {
  // Wait until transmit data register is empty
  while (!(USART2->SR & USART_SR_TXE))
    ;

  // Write data to transmit register
  USART2->DR = data;
}

void uart_send_string(const char *str) {
  // Send each character until null terminator
  while (*str) {
    uart_send_byte(*str++);
  }
}

void uart_send_data(const uint8_t *data, uint16_t len) {
  // Send raw bytes
  for (uint16_t i = 0; i < len; i++) {
    uart_send_byte(data[i]);
  }
}

uint8_t uart_receive_byte(void) {
  // Wait until data is received
  while (!(USART2->SR & USART_SR_RXNE))
    ;

  // Read and return received data
  return (uint8_t)(USART2->DR & 0xFF);
}

int uart_data_available(void) {
  // Check if receive register is not empty
  return (USART2->SR & USART_SR_RXNE) ? 1 : 0;
}

void uart_flush(void) {
  // Read data register to clear any pending data
  while (uart_data_available()) {
    (void)USART2->DR;
  }
}
