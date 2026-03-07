#include "uart.h"
#include "stm32f411xe.h"
#include <stddef.h>

#define UART_BAUD_RATE 115200  // UART2 and UART6 baud rate
#define UART1_BAUD_RATE 500000 // UART1 exact: APB2(100MHz) / BRR(200) = 500000
#define UART_TX_BUFFER_SIZE 256

// Static RX buffers — separate so each instance can have its own size
static volatile uint8_t uart2_rx_buf[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart6_rx_buf[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart1_rx_buf[UART1_RX_BUFFER_SIZE];

// Static TX buffers
static uint8_t uart2_tx_buf[UART_TX_BUFFER_SIZE];
static uint8_t uart6_tx_buf[UART_TX_BUFFER_SIZE];
static uint8_t uart1_tx_buf[UART_TX_BUFFER_SIZE];

// Per-instance UART state
typedef struct {
    volatile uint8_t *rx_buffer; // Pointer to RX circular buffer
    uint16_t rx_buffer_size;     // Must be power of 2
    volatile uint16_t rx_head;   // Write pointer (ISR or DMA position)
    volatile uint16_t rx_tail;   // Read pointer (application)
    uint8_t *tx_buffer;          // Pointer to TX DMA buffer
    volatile uint8_t tx_busy;    // DMA TX in progress
    USART_TypeDef *usart;
    DMA_Stream_TypeDef *dma_tx; // DMA stream for TX
    DMA_Stream_TypeDef *dma_rx; // DMA stream for RX (NULL = interrupt-driven)
} uart_state_t;

static uart_state_t uart_states[UART_INSTANCE_COUNT] = {
    [UART_INSTANCE_2] = {.rx_buffer = uart2_rx_buf,
                         .rx_buffer_size = UART_RX_BUFFER_SIZE,
                         .rx_head = 0,
                         .rx_tail = 0,
                         .tx_busy = 0,
                         .tx_buffer = uart2_tx_buf,
                         .usart = USART2,
                         .dma_tx = DMA1_Stream6,
                         .dma_rx = NULL},
    [UART_INSTANCE_6] = {.rx_buffer = uart6_rx_buf,
                         .rx_buffer_size = UART_RX_BUFFER_SIZE,
                         .rx_head = 0,
                         .rx_tail = 0,
                         .tx_busy = 0,
                         .tx_buffer = uart6_tx_buf,
                         .usart = USART6,
                         .dma_tx = DMA2_Stream6,
                         .dma_rx = NULL},
    [UART_INSTANCE_1] = {.rx_buffer = uart1_rx_buf,
                         .rx_buffer_size = UART1_RX_BUFFER_SIZE,
                         .rx_head = 0,
                         .rx_tail = 0,
                         .tx_busy = 0,
                         .tx_buffer = uart1_tx_buf,
                         .usart = USART1,
                         .dma_tx = DMA2_Stream7,
                         .dma_rx = DMA2_Stream2}};

// Helper: for DMA-RX instances, compute current write position from NDTR
static inline void uart_sync_dma_rx_head(uart_state_t *state) {
    if (state->dma_rx) {
        state->rx_head = state->rx_buffer_size - (uint16_t)state->dma_rx->NDTR;
    }
}

void uart_init(uart_instance_t instance) {
    switch (instance) {
    case UART_INSTANCE_2: {
        // UART2 - LoRa Module (PA2/PA3)

        // Enable GPIOA and USART2 clocks
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        __DSB();

        // Configure PA2 (USART2_TX) and PA3 (USART2_RX) as alternate function
        GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
        GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));

        // Set alternate function to AF7 (USART2) for PA2 and PA3
        GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
        GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));

        // Set output speed to high for better signal integrity
        GPIOA->OSPEEDR |= (3 << (2 * 2)) | (3 << (3 * 2));

        // Configure pull-up on RX pin for noise immunity
        GPIOA->PUPDR &= ~(3 << (3 * 2));
        GPIOA->PUPDR |= (1 << (3 * 2)); // Pull-up on PA3 (RX)

        // APB1 clock is 50MHz (100MHz system clock / 2)
        uint32_t apb1_clock = 50000000;
        USART2->BRR = apb1_clock / UART_BAUD_RATE;

        // Configure USART2
        USART2->CR1 = USART_CR1_UE;
        USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

        // Enable USART2 interrupt in NVIC
        NVIC_SetPriority(USART2_IRQn, 2);
        NVIC_EnableIRQ(USART2_IRQn);

        // Configure DMA1 for USART2 TX (DMA1 Stream 6, Channel 4)
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        __DSB();

        DMA1_Stream6->CR = 0;
        while (DMA1_Stream6->CR & DMA_SxCR_EN)
            ;

        DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                      DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

        DMA1_Stream6->CR = (4 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_DIR_Pos) |
                           DMA_SxCR_MINC | (2 << DMA_SxCR_PL_Pos) |
                           DMA_SxCR_TCIE;

        DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

        NVIC_SetPriority(DMA1_Stream6_IRQn, 2);
        NVIC_EnableIRQ(DMA1_Stream6_IRQn);

        USART2->CR3 |= USART_CR3_DMAT;
        break;
    }

    case UART_INSTANCE_6: {
        // UART6 - Optical Flow Sensor (PA11/PA12)

        // Enable GPIOA and USART6 clocks
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        __DSB();

        // Configure PA11 (USART6_TX) and PA12 (USART6_RX) as alternate function
        GPIOA->MODER &= ~((3 << (11 * 2)) | (3 << (12 * 2)));
        GPIOA->MODER |= (2 << (11 * 2)) | (2 << (12 * 2));

        // Set alternate function to AF8 (USART6) for PA11 and PA12
        GPIOA->AFR[1] &= ~((0xF << ((11 - 8) * 4)) | (0xF << ((12 - 8) * 4)));
        GPIOA->AFR[1] |= (8 << ((11 - 8) * 4)) | (8 << ((12 - 8) * 4));

        // Set output speed to high for better signal integrity
        GPIOA->OSPEEDR |= (3 << (11 * 2)) | (3 << (12 * 2));

        // Configure pull-up on RX pin for noise immunity
        GPIOA->PUPDR &= ~(3 << (12 * 2));
        GPIOA->PUPDR |= (1 << (12 * 2)); // Pull-up on PA12 (RX)

        // APB2 clock is 100MHz (system clock)
        uint32_t apb2_clock = 100000000;
        USART6->BRR = apb2_clock / UART_BAUD_RATE;

        // Configure USART6
        USART6->CR1 = USART_CR1_UE;
        USART6->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

        // Enable USART6 interrupt in NVIC
        NVIC_SetPriority(USART6_IRQn, 2);
        NVIC_EnableIRQ(USART6_IRQn);

        // Configure DMA2 for USART6 TX (DMA2 Stream 6, Channel 5)
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        __DSB();

        DMA2_Stream6->CR = 0;
        while (DMA2_Stream6->CR & DMA_SxCR_EN)
            ;

        DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                      DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

        DMA2_Stream6->CR = (5 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_DIR_Pos) |
                           DMA_SxCR_MINC | (2 << DMA_SxCR_PL_Pos) |
                           DMA_SxCR_TCIE;

        DMA2_Stream6->PAR = (uint32_t)&USART6->DR;

        NVIC_SetPriority(DMA2_Stream6_IRQn, 2);
        NVIC_EnableIRQ(DMA2_Stream6_IRQn);

        USART6->CR3 |= USART_CR3_DMAT;
        break;
    }

    case UART_INSTANCE_1: {
        // UART1 - High-speed 500kbaud device (PA9/PA10)
        // USART1 is on APB2 (100 MHz). BRR = 100,000,000 / 500,000 = 200
        // (exact). RX uses DMA2 Stream 2 Ch4 in circular mode — no per-byte ISR
        // overhead. Idle-line interrupt fires when the bus goes quiet, snapping
        // rx_head.

        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        __DSB();

        // PA9 (USART1_TX) and PA10 (USART1_RX) — AF7
        GPIOA->MODER &= ~((3u << (9 * 2)) | (3u << (10 * 2)));
        GPIOA->MODER |= (2u << (9 * 2)) | (2u << (10 * 2));

        GPIOA->AFR[1] &= ~((0xFu << ((9 - 8) * 4)) | (0xFu << ((10 - 8) * 4)));
        GPIOA->AFR[1] |= (7u << ((9 - 8) * 4)) | (7u << ((10 - 8) * 4));

        // Very high GPIO speed — required for 500kbaud signal integrity
        GPIOA->OSPEEDR |= (3u << (9 * 2)) | (3u << (10 * 2));

        // Pull-up on RX to keep line high when idle
        GPIOA->PUPDR &= ~(3u << (10 * 2));
        GPIOA->PUPDR |= (1u << (10 * 2));

        // 420kbaud: BRR = 238 (exact, no fractional error)
        USART1->BRR = 238;

        // Enable USART1: TE + RE + IDLEIE (idle-line interrupt for DMA RX
        // framing) RXNEIE is intentionally NOT set — DMA handles every byte
        USART1->CR1 =
            USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;

        NVIC_SetPriority(USART1_IRQn, 1); // Higher priority than UART2/6
        NVIC_EnableIRQ(USART1_IRQn);

        // ---- DMA2 Stream 2, Channel 4 — USART1 RX (circular) ----
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        __DSB();

        DMA2_Stream2->CR = 0;
        while (DMA2_Stream2->CR & DMA_SxCR_EN)
            ;

        // Clear all Stream 2 flags (in LIFCR)
        DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                      DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;

        DMA2_Stream2->CR = (4u << DMA_SxCR_CHSEL_Pos) | // Channel 4
                           (0u << DMA_SxCR_DIR_Pos) |   // Peripheral → Memory
                           DMA_SxCR_MINC |           // Increment memory address
                           (2u << DMA_SxCR_PL_Pos) | // High priority
                           DMA_SxCR_CIRC; // Circular mode (never stops)

        DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
        DMA2_Stream2->M0AR = (uint32_t)uart1_rx_buf;
        DMA2_Stream2->NDTR = UART1_RX_BUFFER_SIZE;

        // Start RX DMA immediately — it runs forever in circular mode
        DMA2_Stream2->CR |= DMA_SxCR_EN;

        // ---- DMA2 Stream 7, Channel 4 — USART1 TX ----
        DMA2_Stream7->CR = 0;
        while (DMA2_Stream7->CR & DMA_SxCR_EN)
            ;

        // Clear all Stream 7 flags (in HIFCR)
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |
                      DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

        DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos) | // Channel 4
                           (1u << DMA_SxCR_DIR_Pos) |   // Memory → Peripheral
                           DMA_SxCR_MINC |           // Increment memory address
                           (3u << DMA_SxCR_PL_Pos) | // Very high priority
                           DMA_SxCR_TCIE; // Transfer complete interrupt

        DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

        NVIC_SetPriority(DMA2_Stream7_IRQn, 1);
        NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        // Enable DMA requests from USART1 for both TX and RX
        USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
        break;
    }

    default:
        break;
    }
}

void uart_send_byte(uart_instance_t instance, uint8_t data) {
    uart_state_t *state = &uart_states[instance];

    // Wait for previous transmission to complete
    while (!(state->usart->SR & USART_SR_TC))
        ;

    // Write data to transmit register
    state->usart->DR = data;
}

void uart_send_string(uart_instance_t instance, const char *str) {
    uart_state_t *state = &uart_states[instance];

    // Wait if previous DMA transmission is still in progress
    while (state->tx_busy)
        ;

    // Copy string to TX buffer
    uint16_t len = 0;
    while (str[len] && len < UART_TX_BUFFER_SIZE) {
        state->tx_buffer[len] = str[len];
        len++;
    }

    // Don't send empty strings
    if (len == 0) {
        return;
    }

    // Mark transmission as busy
    state->tx_busy = 1;

    // Configure DMA transfer
    state->dma_tx->M0AR = (uint32_t)state->tx_buffer;
    state->dma_tx->NDTR = len;

    // Enable DMA stream to start transfer
    state->dma_tx->CR |= DMA_SxCR_EN;
}

void uart_send_data(uart_instance_t instance, const uint8_t *data,
                    uint16_t len) {
    uart_state_t *state = &uart_states[instance];

    // Wait if previous DMA transmission is still in progress
    while (state->tx_busy)
        ;

    // Limit to buffer size
    if (len > UART_TX_BUFFER_SIZE) {
        len = UART_TX_BUFFER_SIZE;
    }

    // Don't send empty data
    if (len == 0) {
        return;
    }

    // Copy data to TX buffer
    for (uint16_t i = 0; i < len; i++) {
        state->tx_buffer[i] = data[i];
    }

    // Mark transmission as busy
    state->tx_busy = 1;

    // Configure DMA transfer
    state->dma_tx->M0AR = (uint32_t)state->tx_buffer;
    state->dma_tx->NDTR = len;

    // Enable DMA stream to start transfer
    state->dma_tx->CR |= DMA_SxCR_EN;
}

uint8_t uart_receive_byte(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    uint16_t mask = state->rx_buffer_size - 1;

    // For DMA RX instances, poll NDTR directly so we don't miss bytes
    // that arrived since the last idle-line interrupt.
    if (state->dma_rx) {
        while (state->rx_head == state->rx_tail) {
            state->rx_head =
                state->rx_buffer_size - (uint16_t)state->dma_rx->NDTR;
        }
    } else {
        while (state->rx_head == state->rx_tail)
            ;
    }

    uint8_t data = state->rx_buffer[state->rx_tail];
    state->rx_tail = (state->rx_tail + 1) & mask;
    return data;
}

int uart_data_available(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    uart_sync_dma_rx_head(state);
    return (state->rx_head != state->rx_tail) ? 1 : 0;
}

void uart_flush(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    uart_sync_dma_rx_head(state);
    state->rx_tail = state->rx_head;
}

uint16_t uart_bytes_available(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    uart_sync_dma_rx_head(state);
    uint16_t size = state->rx_buffer_size;
    return (uint16_t)((size + state->rx_head - state->rx_tail) & (size - 1));
}

int uart_tx_busy(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    // Return current transmission status
    return state->tx_busy ? 1 : 0;
}

// USART2 interrupt handler (UART_INSTANCE_2)
void USART2_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_2];
    uint32_t sr = USART2->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART2->DR & 0xFF);
        uint16_t next_head = (state->rx_head + 1) & (state->rx_buffer_size - 1);
        if (next_head != state->rx_tail) {
            state->rx_buffer[state->rx_head] = data;
            state->rx_head = next_head;
        }
    }

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART2->DR;
    }
}

// USART6 interrupt handler (UART_INSTANCE_6)
void USART6_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_6];
    uint32_t sr = USART6->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART6->DR & 0xFF);
        uint16_t next_head = (state->rx_head + 1) & (state->rx_buffer_size - 1);
        if (next_head != state->rx_tail) {
            state->rx_buffer[state->rx_head] = data;
            state->rx_head = next_head;
        }
    }

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART6->DR;
    }
}

// USART1 interrupt handler (UART_INSTANCE_1) — idle-line detection only
// At 500kbaud the DMA handles every byte; this ISR fires when the bus goes
// quiet for one character period, snapping rx_head to the DMA write position.
// Clearing IDLE on F4 requires reading SR then DR (no ICR register available).
// Safe to read DR here because DMA has already consumed the last byte.
void USART1_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_1];
    uint32_t sr = USART1->SR;

    if (sr & USART_SR_IDLE) {
        (void)USART1->DR; // Required to clear the IDLE flag on STM32F4
        state->rx_head = state->rx_buffer_size - (uint16_t)DMA2_Stream2->NDTR;
    }

    // Overrun or framing error — clear by reading DR
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART1->DR;
    }
}

// DMA1 Stream 6 interrupt handler (USART2 TX)
void DMA1_Stream6_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_2];

    // Check for transfer complete
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        // Clear transfer complete flag
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        // Disable DMA stream
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;

        // Mark transmission as complete
        state->tx_busy = 0;
    }

    // Check for transfer error
    if (DMA1->HISR & DMA_HISR_TEIF6) {
        // Clear transfer error flag
        DMA1->HIFCR = DMA_HIFCR_CTEIF6;

        // Disable DMA stream
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;

        // Mark transmission as complete (even on error)
        state->tx_busy = 0;
    }
}

// DMA2 Stream 6 interrupt handler (USART6 TX)
void DMA2_Stream6_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_6];

    if (DMA2->HISR & DMA_HISR_TCIF6) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF6;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        state->tx_busy = 0;
    }

    if (DMA2->HISR & DMA_HISR_TEIF6) {
        DMA2->HIFCR = DMA_HIFCR_CTEIF6;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        state->tx_busy = 0;
    }
}

// DMA2 Stream 7 interrupt handler (USART1 TX)
void DMA2_Stream7_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_1];

    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        state->tx_busy = 0;
    }

    if (DMA2->HISR & DMA_HISR_TEIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTEIF7;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        state->tx_busy = 0;
    }
}
