/**
 ******************************************************************************
 * @file           : rcc.h
 * @author         : Sam C
 * @brief          : Header file for RCC driver for STM32L476RGTx
 ******************************************************************************
 */
#ifndef RCC_H
#define RCC_H

#include <stdint.h>
#include "gpio.h"

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t PLLSAI1CFGR;
    volatile uint32_t PLLSAI2CFGR;
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    uint32_t RESERVED1;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED2;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1RSTR2;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED3;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1ENR1;
    volatile uint32_t APB1ENR2;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;


#define RCC_BASE (0x40021000U)
#define RCC ((RCC_TypeDef *)RCC_BASE)

// Macros
#define SYSCLK_FREQ_HZ    4000000UL      // 4MHz
#define HCLK_FREQ_HZ      SYSCLK_FREQ_HZ // HCLK Prescaler = 1
#define PCLK1_FREQ_HZ     HCLK_FREQ_HZ   // APB1 Prescaler = 1
#define PCLK2_FREQ_HZ     HCLK_FREQ_HZ   // APB2 Prescaler = 1
#define TIM_PCLK_FREQ_HZ  PCLK1_FREQ_HZ  // TIM3 está en APB1

// Prototipos de funciones
void rcc_gpio_clock_enable(GPIO_TypeDef *gpio_port);
void rcc_syscfg_clock_enable(void);
void rcc_usart2_clock_enable(void);
void rcc_tim3_clock_enable(void);

#endif // RCC_H