#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// ===== Parámetros PID =====
float Kp = 1.4, Ki = 0.06, Kd = 0.8;
float dt = 0.1; // intervalo de control en segundos

// ===== Función PID =====
float PID(float setpoint, float y1) {
    static float prev_error = 0.0;
    static float integral = 0.0;
    static float error = 0.0;
    static float derivative = 0.0;

    error = setpoint - y1;
    integral += error * dt;
    derivative = (error - prev_error) / dt;
    float u1 = Kp * error + Ki * integral + Kd * derivative;

    if (integral > 250) integral = 250;
    if (integral < -250) integral = -250;

    // Saturar salida (0-250 para PWM)
    if (u1 > 250) u1 = 250;
    if (u1 < 0) u1 = 0;

    prev_error = error;
    printf("Error=%.2f  Integral=%.2f  Derivativo=%.2f\n", error, integral, derivative);
    printf("----------------------------\n");
    return u1;
}

int main() {
    stdio_init_all();

    // ===== Configurar ADC =====
    adc_init();
    adc_gpio_init(27); // GPIO27 -> ADC1 (Sensor 1)

    // ===== Configurar PWM =====
    gpio_set_function(0, GPIO_FUNC_PWM); // GPIO0 salida 1 PWM
    uint slice_num = pwm_gpio_to_slice_num(0);
    pwm_set_wrap(slice_num, 250);  // rango 0-250
    pwm_set_clkdiv(slice_num, 250); // Divisor de reloj
    pwm_set_enabled(slice_num, true);

    // ===== Bucle principal =====
    while (true) {
        // Leer setpoint (ADC0)

        float setpoint = 126.0; // Valor fijo para pruebas

        adc_select_input(1);
        float raw_y1 = adc_read();
        float y1 = (raw_y1 / 4095.0f) * 250.0f;

        // Calcular PID
        float u1 = PID(setpoint, y1);

        // Aplicar al PWM
        pwm_set_gpio_level(0, (uint16_t)u1);

        // Debug por USB
        printf("SP=%.2f  Entrada=%.2f  Salida=%.2f\n", setpoint, y1, u1);
        printf("Sensado=%.2f\n", raw_y1);

        sleep_ms(100); // 100 ms → dt = 0.1s
    }  
}