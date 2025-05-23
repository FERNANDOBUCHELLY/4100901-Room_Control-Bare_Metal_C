/**
 ******************************************************************************
 * @file           : uart.c
 * @author         : Sam C
 * @brief          : UART driver for STM32L476RGTx
 ******************************************************************************
 */
#include "uart.h"
#include "rcc.h"  // Para rcc_usart2_clock_enable y PCLK1_FREQ_HZ
#include "gpio.h" // Para configurar pines PA2, PA3


// USART_ISR bits
#define USART_ISR_TXE_Pos       7U  // Transmit Data Register Empty
#define USART_ISR_TXE           (1UL << USART_ISR_TXE_Pos)
#define USART_ISR_RXNE_Pos      5U  // Read Data Register Not Empty
#define USART_ISR_RXNE          (1UL << USART_ISR_RXNE_Pos)


void uart2_init(uint32_t baud_rate)
{
    // 1. Configurar pines PA2 (TX) y PA3 (RX) como Alternate Function (AF7)
    gpio_setup_pin(GPIOA, 2, GPIO_MODE_AF, 7);
    gpio_setup_pin(GPIOA, 3, GPIO_MODE_AF, 7);

    // 2. Habilitar el reloj para USART2
    rcc_usart2_clock_enable();

    // 3. Configurar USART2
    //    Deshabilitar USART antes de configurar (importante si se reconfigura)
    USART2->CR1 &= ~(0x01 << 0);

    // Configurar Baud Rate (USARTDIV en BRR)
    // USARTDIV = fCK_USART / BaudRate
    uint32_t usart_div = (PCLK1_FREQ_HZ + (baud_rate / 2U)) / baud_rate; // Con redondeo
    USART2->BRR = usart_div;

    // Habilitar Transmisor (TE) y Receptor (RE)
    USART2->CR1 |= (0x01 << 2 | 0x01 << 3);

    // Finalmente, habilitar USART (UE bit en CR1)
    USART2->CR1 |= 0x01 << 0;
}

void uart2_send_char(char c)
{
    // Esperar hasta que el buffer de transmisión (TDR) esté vacío.
    // Se verifica el flag TXE (Transmit data register empty) en el registro ISR.
    while (!(USART2->ISR & USART_ISR_TXE));

    // Escribir el dato en el Transmit Data Register (TDR).
    USART2->TDR = (uint8_t)c;
}

void uart2_send_string(const char *str)
{
    while (*str != '\0') {
        uart2_send_char(*str);
        str++;
    }
}

/**
 * @brief Handler de interrupción para USART2
 *        Este handler se llama cuando hay datos recibidos en USART2.
 *        Procesa el dato recibido y lo envía de vuelta (eco).
 */
//void USART2_IRQHandler(void)
//{
    // Verificar si la interrupción fue por RXNE (dato recibido y RDR no vacío)
    //if (USART2->ISR & USART_ISR_RXNE) {
        // Leer el dato del RDR. Esta acción usualmente limpia el flag RXNE.
        //char received_char = (char)(USART2->RDR & 0xFF);
        //uart2_send_char(received_char); // Eco del carácter recibido 
        // Procesar el carácter recibido.
    //}
//}
