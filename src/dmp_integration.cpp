// DMP 集成文件 - 为 Arduino 平台适配 InvenSense DMP 库

#define ARDUINO_HAL
#define MPU6050

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"

// ============================================================================
// HAL 函数实现
// ============================================================================

static uint8_t hal_mpu_i2c_addr = MPU6050_I2C_ADDR;

// 设置 MPU I2C 地址
extern "C" void hal_set_mpu_addr(uint8_t addr) {
    hal_mpu_i2c_addr = addr;
}

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

void delay_ms(unsigned long ms) {
    delay(ms);
}

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

// 前向声明
struct int_param_s;

int reg_int_cb(struct int_param_s *int_param) {
    return 0;  // 不使用中断
}

// Arduino NOP 函数
void __no_operation(void) {
    asm("nop");  // 空操作指令
}

// 日志函数（可选）
#define log_i(...)  do {} while (0)
#define log_e(...)  do {} while (0)

// 宏定义
#define min(a,b) ((a<b)?(a):(b))

// ============================================================================
// DMP 库源码包含（使用 extern "C" 包装）
// ============================================================================

extern "C" {
    // 定义 Arduino 平台标记
    #define ARDUINO_PLATFORM 1
    #define MAX_PACKET_LENGTH (32)  // 使用 inv_mpu_dmp_motion_driver.c 的值

    #include "../DMP/inv_mpu.c"
    #include "../DMP/inv_mpu_dmp_motion_driver.c"
}
