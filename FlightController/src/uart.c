#include "uart.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"

// LoRa module baud rate - change if your module uses different rate
#define LORA_BAUD_RATE 115200

// Circular buffer for RX data
static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0; // Write pointer (updated in ISR)
static volatile uint16_t rx_tail = 0; // Read pointer (updated in main code)

void uart_init(void) {
  // Enable GPIOA and USART2 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  __DSB();

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
  USART2->BRR = apb1_clock / LORA_BAUD_RATE;

  // Configure USART2 with proper enable sequence:
  // 1. First enable USART
  USART2->CR1 = USART_CR1_UE; // Enable USART first

  // 2. Then enable transmitter and receiver
  USART2->CR1 |= USART_CR1_TE |    // Transmitter enable
                 USART_CR1_RE |    // Receiver enable
                 USART_CR1_RXNEIE; // RX interrupt enable

  // Enable USART2 interrupt in NVIC
  NVIC_SetPriority(USART2_IRQn, 2); // Priority 2 (lower than IMU)
  NVIC_EnableIRQ(USART2_IRQn);
}

void uart_send_byte(uint8_t data) {
  // Wait for previous transmission to complete
  while (!(USART2->SR & USART_SR_TC))
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
  // Wait until data is available in buffer
  while (rx_head == rx_tail)
    ;

  // Read byte from buffer
  uint8_t data = rx_buffer[rx_tail];
  rx_tail = (rx_tail + 1) & (UART_RX_BUFFER_SIZE - 1);

  return data;
}

int uart_data_available(void) {
  // Check if buffer has data
  return (rx_head != rx_tail) ? 1 : 0;
}

void uart_flush(void) {
  // Clear the circular buffer
  rx_tail = rx_head;
}

uint16_t uart_bytes_available(void) {
  // Calculate bytes in buffer (handles wraparound)
  return (uint16_t)((UART_RX_BUFFER_SIZE + rx_head - rx_tail) &
                    (UART_RX_BUFFER_SIZE - 1));
}

// USART2 interrupt handler
void USART2_IRQHandler(void) {
  uint32_t sr = USART2->SR;

  // Check for RX not empty
  if (sr & USART_SR_RXNE) {
    uint8_t data = (uint8_t)(USART2->DR & 0xFF);

    // Calculate next head position
    uint16_t next_head = (rx_head + 1) & (UART_RX_BUFFER_SIZE - 1);

    // Store data if buffer not full (drop if full)
    if (next_head != rx_tail) {
      rx_buffer[rx_head] = data;
      rx_head = next_head;
    }
    // else: buffer overflow, data dropped
  }

  // Clear error flags by reading SR then DR
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
    (void)USART2->DR; // Dummy read to clear error flags
  }
}
