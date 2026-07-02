#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

// === 你的国产 MPU6050 I2C 地址 ===
#define MPU6050_ADDR 0x70

MPU6050 mpu;

// I2C 总线扫描（找不到传感器时自动触发）
void scanI2C() {
    Serial.println(">> 扫描 I2C 总线...");
    byte count = 0;
    for (byte a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("   发现设备: 0x%02X\n", a);
            count++;
        }
    }
    if (count == 0)
        Serial.println("   未发现任何 I2C 设备，请检查接线！");
    Serial.printf(">> 扫描完成，共 %d 个设备\n", count);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=============================");
    Serial.println("  ESP8266 + MPU6050 角度读取");
    Serial.println("  互补滤波 (非 DMP)");
    Serial.println("=============================\n");

    if (!mpu.begin(MPU6050_ADDR)) {
        Serial.printf("[错误] 0x%02X 未找到 MPU6050！\n", MPU6050_ADDR);
        scanI2C();
        Serial.println("程序暂停。请检查接线和地址后重启。");
        while (true) { delay(1000); }
    }

    Serial.printf("[OK] MPU6050 已连接  地址=0x%02X  WHO_AM_I=0x%02X\n",
                  mpu.getAddress(), mpu.getWhoAmI());

    Serial.println("\n>> 陀螺仪校准中，请保持传感器静止...");
    mpu.calibrateGyro(1000);
    float ox, oy, oz;
    mpu.getGyroOffsets(ox, oy, oz);
    Serial.printf("   零偏: X=%.2f  Y=%.2f  Z=%.2f  (°/s)\n", ox, oy, oz);
    Serial.println(">> 校准完成，开始输出角度数据\n");

    Serial.println("Pitch(°)   Roll(°)    Yaw(°)     Temp(°C)");
    Serial.println("--------------------------------------------");
}

void loop() {
    mpu.update();

    Serial.printf("%-10.2f %-10.2f %-10.2f %.1f\n",
                  mpu.getPitch(), mpu.getRoll(), mpu.getYaw(),
                  mpu.getTemperature());

    delay(50);  // ~20 Hz 输出
}
