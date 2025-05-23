/**
 ******************************************************************************
 * @file           : systick.h
 * @author         : Sam C
 * @brief          : Header file for SysTick driver for STM32L476RGTx
 ******************************************************************************
 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CTRL;  // Control and Status Register, Offset: 0x00
    volatile uint32_t LOAD;  // Reload Value Register,    Offset: 0x04
    volatile uint32_t VAL;   // Current Value Register,   Offset: 0x08
    volatile uint32_t CALIB; // Calibration Register,     Offset: 0x0C
} SysTick_TypeDef;


#define SYSTICK_BASE        (0xE000E010UL)
#define SysTick             ((SysTick_TypeDef *)SYSTICK_BASE)

// Prototipos de funciones
void systick_init_1ms(void);
uint32_t systick_get_tick(void);
void systick_delay_ms(uint32_t ms);

#endif // SYSTICK_H
