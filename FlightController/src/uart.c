#include "uart.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"

// UART baud rate - same for both instances
#define UART_BAUD_RATE 115200

// TX DMA buffer size
#define UART_TX_BUFFER_SIZE 256

// Per-instance UART state
typedef struct {
    volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;  // Write pointer (updated in ISR)
    volatile uint16_t rx_tail;  // Read pointer (updated in main code)
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    volatile uint8_t tx_busy;   // Flag to indicate DMA transmission in progress
    USART_TypeDef *usart;       // USART peripheral
    DMA_Stream_TypeDef *dma_stream;  // DMA stream for TX
} uart_state_t;

// UART instance states
static uart_state_t uart_states[UART_INSTANCE_COUNT] = {
    [UART_INSTANCE_2] = {
        .rx_head = 0,
        .rx_tail = 0,
        .tx_busy = 0,
        .usart = USART2,
        .dma_stream = DMA1_Stream6
    },
    [UART_INSTANCE_6] = {
        .rx_head = 0,
        .rx_tail = 0,
        .tx_busy = 0,
        .usart = USART6,
        .dma_stream = DMA2_Stream6
    }
};

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

            // Calculate baud rate register value
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
            while (DMA1_Stream6->CR & DMA_SxCR_EN);

            // Clear all interrupt flags for Stream 6
            DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                          DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

            DMA1_Stream6->CR = (4 << DMA_SxCR_CHSEL_Pos) |
                               (1 << DMA_SxCR_DIR_Pos) |
                               DMA_SxCR_MINC |
                               (2 << DMA_SxCR_PL_Pos) |
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

            // Calculate baud rate register value
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
            while (DMA2_Stream6->CR & DMA_SxCR_EN);

            // Clear all interrupt flags for Stream 6
            DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                          DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

            DMA2_Stream6->CR = (5 << DMA_SxCR_CHSEL_Pos) |
                               (1 << DMA_SxCR_DIR_Pos) |
                               DMA_SxCR_MINC |
                               (2 << DMA_SxCR_PL_Pos) |
                               DMA_SxCR_TCIE;

            DMA2_Stream6->PAR = (uint32_t)&USART6->DR;

            NVIC_SetPriority(DMA2_Stream6_IRQn, 2);
            NVIC_EnableIRQ(DMA2_Stream6_IRQn);

            USART6->CR3 |= USART_CR3_DMAT;
            break;
        }

        default:
            break;
    }
}

void uart_send_byte(uart_instance_t instance, uint8_t data) {
    uart_state_t *state = &uart_states[instance];

    // Wait for previous transmission to complete
    while (!(state->usart->SR & USART_SR_TC));

    // Write data to transmit register
    state->usart->DR = data;
}

void uart_send_string(uart_instance_t instance, const char *str) {
    uart_state_t *state = &uart_states[instance];

    // Wait if previous DMA transmission is still in progress
    while (state->tx_busy);

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
    state->dma_stream->M0AR = (uint32_t)state->tx_buffer;
    state->dma_stream->NDTR = len;

    // Enable DMA stream to start transfer
    state->dma_stream->CR |= DMA_SxCR_EN;
}

void uart_send_data(uart_instance_t instance, const uint8_t *data, uint16_t len) {
    uart_state_t *state = &uart_states[instance];

    // Wait if previous DMA transmission is still in progress
    while (state->tx_busy);

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
    state->dma_stream->M0AR = (uint32_t)state->tx_buffer;
    state->dma_stream->NDTR = len;

    // Enable DMA stream to start transfer
    state->dma_stream->CR |= DMA_SxCR_EN;
}

uint8_t uart_receive_byte(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];

    // Wait until data is available in buffer
    while (state->rx_head == state->rx_tail);

    // Read byte from buffer
    uint8_t data = state->rx_buffer[state->rx_tail];
    state->rx_tail = (state->rx_tail + 1) & (UART_RX_BUFFER_SIZE - 1);

    return data;
}

int uart_data_available(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    // Check if buffer has data
    return (state->rx_head != state->rx_tail) ? 1 : 0;
}

void uart_flush(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    // Clear the circular buffer
    state->rx_tail = state->rx_head;
}

uint16_t uart_bytes_available(uart_instance_t instance) {
    uart_state_t *state = &uart_states[instance];
    // Calculate bytes in buffer (handles wraparound)
    return (uint16_t)((UART_RX_BUFFER_SIZE + state->rx_head - state->rx_tail) &
                      (UART_RX_BUFFER_SIZE - 1));
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

    // Check for RX not empty
    if (sr & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART2->DR & 0xFF);

        // Calculate next head position
        uint16_t next_head = (state->rx_head + 1) & (UART_RX_BUFFER_SIZE - 1);

        // Store data if buffer not full (drop if full)
        if (next_head != state->rx_tail) {
            state->rx_buffer[state->rx_head] = data;
            state->rx_head = next_head;
        }
    }

    // Clear error flags by reading SR then DR
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART2->DR;
    }
}

// USART6 interrupt handler (UART_INSTANCE_6)
void USART6_IRQHandler(void) {
    uart_state_t *state = &uart_states[UART_INSTANCE_6];
    uint32_t sr = USART6->SR;

    // Check for RX not empty
    if (sr & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART6->DR & 0xFF);

        // Calculate next head position
        uint16_t next_head = (state->rx_head + 1) & (UART_RX_BUFFER_SIZE - 1);

        // Store data if buffer not full (drop if full)
        if (next_head != state->rx_tail) {
            state->rx_buffer[state->rx_head] = data;
            state->rx_head = next_head;
        }
    }

    // Clear error flags by reading SR then DR
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART6->DR;
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

    // Check for transfer complete
    if (DMA2->HISR & DMA_HISR_TCIF6) {
        // Clear transfer complete flag
        DMA2->HIFCR = DMA_HIFCR_CTCIF6;

        // Disable DMA stream
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;

        // Mark transmission as complete
        state->tx_busy = 0;
    }

    // Check for transfer error
    if (DMA2->HISR & DMA_HISR_TEIF6) {
        // Clear transfer error flag
        DMA2->HIFCR = DMA_HIFCR_CTEIF6;

        // Disable DMA stream
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;

        // Mark transmission as complete (even on error)
        state->tx_busy = 0;
    }
}
