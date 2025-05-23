/**
 ******************************************************************************
 * @file           : gpio.c
 * @author         : Sam C
 * @brief          : GPIO driver for STM32L476RGTx
 ******************************************************************************
 */
#include "gpio.h"
#include "rcc.h"
#include "nvic.h"

void gpio_setup_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number,
                    uint8_t mode, uint8_t alternate_function)
{
    // 1. Habilitar el reloj para el puerto GPIO correspondiente
    //    La función rcc_gpio_clock_enable se encarga de esto.
    rcc_gpio_clock_enable(gpio_port);

    // 2. Configurar el modo del pin (Input, Output, AF, Analog)
    //    Cada pin usa 2 bits en MODER. Limpiar y luego establecer.
    gpio_port->MODER &= ~(0x03U << (pin_number * 2)); // Limpiar los 2 bits del pin
    gpio_port->MODER |= (mode << (pin_number * 2));   // Establecer el modo

    // 3. Configurar la función alternativa
    //    Solo si es modo AF. Cada pin usa 4 bits.
    //    Pines 0-7 usan AFRL, pines 8-15 usan AFRH.
    if (mode == GPIO_MODE_AF) {
        uint32_t temp_af_val = alternate_function;
        if (pin_number < 8) { // AFRL
            gpio_port->AFRL &= ~(0x0FU << (pin_number * 4));          // Limpiar los 4 bits del pin
            gpio_port->AFRL |= (temp_af_val << (pin_number * 4)); // Establecer AF
        } else { // AFRH
            gpio_port->AFRH &= ~(0x0FU << ((pin_number - 8) * 4));       // Limpiar los 4 bits del pin
            gpio_port->AFRH |= (temp_af_val << ((pin_number - 8) * 4)); // Establecer AF
        }
    }
}

void gpio_write_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number, uint8_t pin_state)
{
    if (pin_state == GPIO_PIN_SET) {
        // Establecer el bit correspondiente en BSRR (parte baja para SET)
        gpio_port->BSRR = (1U << pin_number);
    } else {
        // Establecer el bit correspondiente en BSRR (parte alta para RESET)
        gpio_port->BSRR = (1U << (pin_number + 16));
    }
}

void gpio_toggle_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    // Alternar el estado del pin usando ODR (Output Data Register)
    gpio_port->ODR ^= (1U << pin_number);
}

uint8_t gpio_read_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    // Leer el bit correspondiente del Input Data Register (IDR)
    if ((gpio_port->IDR >> pin_number) & 0x01U) {
        return GPIO_PIN_SET;
    } else {
        return GPIO_PIN_RESET;
    }
}

/**
 * @brief Rutina de Servicio de Interrupción para EXTI líneas 10 a 15.
 *        Este nombre debe coincidir exactamente con el definido en la tabla de vectores
 *        del archivo de arranque (startup_stm32l476rgtx.s).
 *        Esta ISR puede ser llamada por room_control.c si la lógica es compleja.
 */
//void EXTI15_10_IRQHandler(void) {
    // 1. Verificar si la interrupción fue de la línea EXTI13
    //if ((EXTI->PR1 & (1U << 13)) != 0) {
        // 2. Limpiar el flag de pendiente de la interrupción (escribiendo '1')
        //EXTI->PR1 |= (1U << 13);
        // 3. Procesar boton presionado
    //}
//}

