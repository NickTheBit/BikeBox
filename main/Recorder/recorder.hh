/* File: recorder.hh
 * Author: Nick Gkloumpos
*/
#pragma once

/* structure to keep latest accelerometer/positional sensor data */
struct SensorFrame {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

extern "C" void recorderTask(void *);
void Configure_GPIO_Interrupt(void);
void IRAM_ATTR gpio_isr_handler(uint32_t arg);
