/**
 ******************************************************************************
 * @file           : systick.c
 * @author         : Sam C
 * @brief          : SysTick driver for STM32L476RGTx
 ******************************************************************************
 */
#include "systick.h"
#include "rcc.h" // Para SYSCLK_FREQ_HZ


// Variable global para contar los ticks de milisegundos
// 'volatile' es importante porque esta variable es modificada en una ISR
// y leída en el flujo principal del programa.
static volatile uint32_t g_systick_ms_count = 0;


void systick_init_1ms(void)
{
    // 1. Calcular el valor de recarga para 1 ms
    uint32_t reload_value = (SYSCLK_FREQ_HZ / 1000U) - 1U; // (4000000 / 1000) - 1 = 3999

    // 2. Configurar el registro de recarga (SysTick_LOAD)
    SysTick->LOAD = reload_value & 0x00FFFFFFUL;

    // 3. Poner a cero el valor actual del temporizador (SysTick_VAL)
    SysTick->VAL = 0x00000000UL;

    // 4. Configurar el registro de control (SysTick_CTRL)
    SysTick->CTRL = (0x01 << 2) | // Usa reloj del procesador (HCLK)
                    (0x01 << 1) | // Habilita interrupción de SysTick
                    (0x01 << 0) ; // Habilita el contador SysTick
}

uint32_t systick_get_tick(void)
{
    return g_systick_ms_count;
}


void systick_delay_ms(uint32_t ms)
{
    uint32_t start_tick = systick_get_tick(); 
    while ((systick_get_tick() - start_tick) < ms) {
        // Espera hasta que hayan transcurridos los milisegundos deseados
    }
}


void SysTick_Handler(void)
{
    g_systick_ms_count++; // Incrementar el contador de ticks global
}

