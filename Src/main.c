/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sam C
 * @brief          : Main program body
 ******************************************************************************
 */

#include "gpio.h"
#include "systick.h"
#include "nvic.h"
#include "uart.h"
#include "tim.h"
#include "room_control.h"

// --------- AÑADE ESTA LÍNEA ----------
#define USART_ISR_RXNE (1U << 5)
// -------------------------------------

#define BUTTON_PIN             13
#define BUTTON_PORT            GPIOC
#define EXTERNAL_LED_PORT      GPIOA
#define EXTERNAL_LED_PIN       7         // D12 --> PWM (PA7)
#define HEARTBEAT_LED_PORT     GPIOA
#define HEARTBEAT_LED_PIN      5
// NO redefinir EXTERNAL_LED_ONOFF_PIN aquí si ya está en gpio.h

volatile uint8_t led_timer_active = 0;
volatile uint32_t led_on_timestamp = 0;

/**
 * @brief Parpadea el LED heartbeat cada 500 ms.
 */
void heartbeat_led_toggle(void)
{
    static uint32_t last_tick = 0;
    if (systick_get_tick() - last_tick >= 500) {
        gpio_toggle_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        last_tick = systick_get_tick();
    }
}

/**
 * @brief Verifica si deben haberse cumplido los 3 segundos para apagar el LED externo.
 */
void external_led_handler(void)
{
    if (led_timer_active && (systick_get_tick() - led_on_timestamp >= 3000)) {
        gpio_write_pin(EXTERNAL_LED_PORT, EXTERNAL_LED_PIN, 0); // Apagar LED
        led_timer_active = 0;

        // Enviar mensaje al monitor serial
        uart2_send_string("LED apagado\r\n");
    }
}

/**
 * @brief Manejador de interrupción EXTI para PC13 (botón de usuario).
 */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR1 & (1U << BUTTON_PIN)) != 0) {
        // Limpiar bandera de interrupción
        EXTI->PR1 |= (1U << BUTTON_PIN);

        // Verificar si el botón está presionado (activo bajo)
        if (gpio_read_pin(BUTTON_PORT, BUTTON_PIN) == 0) {
            gpio_write_pin(EXTERNAL_LED_PORT, EXTERNAL_LED_PIN, 1); // Encender LED
            led_on_timestamp = systick_get_tick();
            led_timer_active = 1;

            // Enviar mensajes al monitor serial
            uart2_send_string("Boton presionado\r\n");
            uart2_send_string("LED encendido\r\n");
        }
    }
}

/**
 * @brief Manejador de interrupción para USART2:
 *   - 'J' = máxima intensidad PWM,
 *   - 'H' = 20% intensidad PWM,
 *   - 'T' = Toggle ON/OFF LED.
 */
void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE)
    {
        char c = (char)(USART2->RDR); // Leer el dato recibido

        static uint8_t led_state = 0; // Para toggle 'T'

        if (c == 'J') {
            tim3_ch1_pwm_set_duty_cycle(100); // 100% intensidad
            uart2_send_string("Intensidad al maximo\r\n");
        } else if (c == 'H') {
            tim3_ch1_pwm_set_duty_cycle(20); // 20% intensidad
            uart2_send_string("Intensidad al 20%\r\n");
        } else if (c == 'T') {
            led_state = !led_state;
            gpio_write_pin(EXTERNAL_LED_PORT, EXTERNAL_LED_PIN, led_state);
            if (led_state)
                uart2_send_string("LED ENCENDIDO\r\n");
            else
                uart2_send_string("LED APAGADO\r\n");
        }
    }
}

/**
 * @brief Función principal del programa.
 */
int main(void)
{
    // Inicialización del SysTick a 1 ms
    systick_init_1ms(); // Usa SYSCLK_FREQ_HZ de rcc.h

    // Inicializar LED Heartbeat (PA5)
    gpio_setup_pin(GPIOA, HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT, 0);

    // Inicializar LED externo ON/OFF (PA6)
    gpio_setup_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, GPIO_MODE_OUTPUT, 0);

    // Inicializar LED controlado por botón (PA7, D12, PWM)
    gpio_setup_pin(EXTERNAL_LED_PORT, EXTERNAL_LED_PIN, GPIO_MODE_OUTPUT, 0);

    // Inicializar botón (PC13) con EXTI
    gpio_setup_pin(GPIOC, BUTTON_PIN, GPIO_MODE_INPUT, 0);
    nvic_exti_pc13_button_enable();  // Habilita interrupción EXTI para PC13

    // Inicializar USART2
    uart2_init(115200);
    nvic_usart2_irq_enable();

    // Inicializar TIM3 CH1 para PWM (LED D12, PA7)
    tim3_ch1_pwm_init(1000);             // Frecuencia: 1000 Hz
    tim3_ch1_pwm_set_duty_cycle(50);     // Duty cycle inicial: 50%

    // Inicializar lógica de la aplicación
    room_control_app_init();

    uart2_send_string("\r\nSistema Inicializado. Esperando eventos...\r\n");

    while (1) {
        heartbeat_led_toggle();      // Parpadeo LED heartbeat
        external_led_handler();      // Apagado automático del LED externo tras 3s
    }
}
