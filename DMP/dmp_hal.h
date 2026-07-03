// HAL 适配层 - 为 Arduino 平台提供 DMP 库所需的接口
#ifndef DMP_HAL_H
#define DMP_HAL_H

#include <Arduino.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Wire 对象前向声明 ----
#include <Wire.h>

// ---- HAL 全局变量 ----
static uint8_t hal_mpu_i2c_addr = 0x68;

// ---- HAL I2C 函数 ----
int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char const *data) {
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.write(data, length);
    return Wire.endTransmission() == 0 ? 0 : -1;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data) {
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return -1;

    Wire.requestFrom((uint8_t)slave_addr, (uint8_t)length);
    for (int i = 0; i < length; i++) {
        if (!Wire.available()) return -1;
        data[i] = Wire.read();
    }
    return 0;
}

// ---- HAL 延时函数 ----
void delay_ms(unsigned long ms) {
    delay(ms);
}

// ---- HAL 时间函数 ----
static unsigned long g_ms_ticks = 0;
static unsigned long last_ms_ticks = 0;

void get_ms(unsigned long *count) {
    if (!count) return;
    unsigned long now = millis();
    if (now < last_ms_ticks) {
        g_ms_ticks = 0;
    }
    g_ms_ticks += (now - last_ms_ticks);
    last_ms_ticks = now;
    *count = g_ms_ticks;
}

// ---- HAL 中断回调（不使用）----
int reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin) {
    return 0;
}

// ---- HAL 数学函数 ----
// labs 和 fabsf 已经在 Arduino 中可用

#ifdef __cplusplus
}
#endif

#endif // DMP_HAL_H
