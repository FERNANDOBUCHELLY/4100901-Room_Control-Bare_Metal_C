/**
 ******************************************************************************
 * @file           : rcc.c
 * @author         : Sam C
 * @brief          : RCC driver for STM32L476RGTx
 ******************************************************************************
 */
#include "rcc.h"

void rcc_gpio_clock_enable(GPIO_TypeDef *gpio_port)
{
    if (gpio_port == GPIOA) {
        RCC->AHB2ENR |= 0x01 << 0;
    } else if (gpio_port == GPIOB) {
        RCC->AHB2ENR |= 0x01 << 1;
    } else if (gpio_port == GPIOC) {
        RCC->AHB2ENR |= 0x01 << 2;
    }
    // Añadir más puertos GPIO si son necesarios (D, E, F, G, H)
}

void rcc_syscfg_clock_enable(void)
{
    RCC->APB2ENR |= 0x01 << 0; // SYSCFG clock enable
}

void rcc_usart2_clock_enable(void)
{
    RCC->APB1ENR1 |= 0x01 << 17; // USART2 clock enable
}

void rcc_tim3_clock_enable(void)
{
    RCC->APB1ENR1 |= 0x01 << 1; // TIM3 clock enable
}