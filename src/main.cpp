#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "MPU6050_DMP.h"

MPU6050_DMP mpu;

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
    Serial.println("  使用 DMP (数字运动处理器)");
    Serial.println("=============================\n");

    if (!mpu.begin(MPU6050_I2C_ADDR)) {
        Serial.printf("[错误] 0x%02X 未找到 MPU6050！\n", MPU6050_I2C_ADDR);
        scanI2C();
        Serial.println("程序暂停。请检查接线和地址后重启。");
        while (true) { delay(1000); }
    }

    Serial.printf("[OK] MPU6050 已连接  地址=0x%02X\n", mpu.getAddress());

    Serial.println("\n>> DMP 初始化完成，开始读取角度和加速度数据\n");

    Serial.println("Pitch(°)   Roll(°)    Yaw(°)     AccX(g)    AccY(g)    AccZ(g)");
    Serial.println("--------------------------------------------------------------------");
}

void loop() {
    mpu.update();

    Serial.printf("%-10.2f %-10.2f %-10.2f %-10.3f %-10.3f %-10.3f\n",
                  mpu.getPitch(), mpu.getRoll(), mpu.getYaw(),
                  mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());

    delay(50);  // ~20 Hz 输出
}
