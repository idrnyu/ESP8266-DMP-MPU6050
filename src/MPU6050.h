#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ---- MPU6050 寄存器地址 ----
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C
#define MPU6050_WHO_AM_I     0x75

class MPU6050 {
public:
    // 初始化传感器，返回是否成功
    // accelRange: 0=±2g  1=±4g  2=±8g  3=±16g
    // gyroRange:  0=±250  1=±500  2=±1000  3=±2000  (°/s)
    bool begin(uint8_t address, uint8_t accelRange = 0, uint8_t gyroRange = 0) {
        addr = address;
        Wire.begin();

        whoAmI = readReg(MPU6050_WHO_AM_I);
        if (whoAmI == 0xFF || whoAmI == 0x00) return false;

        // 唤醒传感器（退出睡眠模式）
        writeReg(MPU6050_PWR_MGMT_1, 0x00);
        delay(100);

        // 采样率分频器 = 0 → 1 kHz
        writeReg(MPU6050_SMPLRT_DIV, 0x00);

        // 低通滤波 DLPF_CFG=3 → 带宽 42 Hz，内部采样率 1 kHz
        writeReg(MPU6050_CONFIG, 0x03);

        // 加速度计量程
        uint8_t afs = accelRange & 0x03;
        writeReg(MPU6050_ACCEL_CONFIG, afs << 3);
        accelScale = 16384.0f / (1 << afs);   // LSB/g

        // 陀螺仪量程
        uint8_t gfs = gyroRange & 0x03;
        writeReg(MPU6050_GYRO_CONFIG, gfs << 3);
        gyroScale = 131.0f / (1 << gfs);       // LSB/(°/s)

        prevMicros = micros();
        return true;
    }

    // 陀螺仪校准——校准期间保持传感器绝对静止
    void calibrateGyro(uint16_t samples = 1000) {
        float sx = 0, sy = 0, sz = 0;
        for (uint16_t i = 0; i < samples; i++) {
            readAllData();
            sx += (float)rawGyroX / gyroScale;
            sy += (float)rawGyroY / gyroScale;
            sz += (float)rawGyroZ / gyroScale;
            delay(2);
        }
        gyroXOffset = sx / samples;
        gyroYOffset = sy / samples;
        gyroZOffset = sz / samples;
    }

    // 读取数据并更新互补滤波
    void update() {
        readAllData();

        // 原始值 → 物理量
        accelX = (float)rawAccX / accelScale;   // g
        accelY = (float)rawAccY / accelScale;
        accelZ = (float)rawAccZ / accelScale;

        gyroX = (float)rawGyroX / gyroScale - gyroXOffset;  // °/s
        gyroY = (float)rawGyroY / gyroScale - gyroYOffset;
        gyroZ = (float)rawGyroZ / gyroScale - gyroZOffset;

        temperature = (float)rawTemp / 340.0f + 36.53f;     // °C

        // 时间步长（秒）
        uint32_t now = micros();
        float dt = (now - prevMicros) / 1000000.0f;
        prevMicros = now;

        // 加速度计角度（绝对参考，受重力方向约束）
        float accRoll  = atan2(accelY, accelZ) * 180.0f / PI;
        float accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;

        // 陀螺仪积分
        roll  += gyroX * dt;
        pitch += gyroY * dt;
        yaw   += gyroZ * dt;

        // 互补滤波：98% 陀螺仪 + 2% 加速度计
        roll  = alpha * roll  + (1.0f - alpha) * accRoll;
        pitch = alpha * pitch + (1.0f - alpha) * accPitch;
        // yaw 无加速度计参考，会随时间漂移
    }

    // 角度输出（°）
    float getPitch() const { return pitch; }
    float getRoll()  const { return roll; }
    float getYaw()   const { return yaw; }

    // 加速度输出（g）
    float getAccX() const { return accelX; }
    float getAccY() const { return accelY; }
    float getAccZ() const { return accelZ; }

    // 角速度输出（°/s）
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }

    float getTemperature() const { return temperature; }
    uint8_t getWhoAmI() const { return whoAmI; }
    uint8_t getAddress() const { return addr; }

    void getGyroOffsets(float &ox, float &oy, float &oz) {
        ox = gyroXOffset; oy = gyroYOffset; oz = gyroZOffset;
    }

private:
    uint8_t addr = 0x70;
    uint8_t whoAmI = 0;

    int16_t rawAccX = 0, rawAccY = 0, rawAccZ = 0;
    int16_t rawGyroX = 0, rawGyroY = 0, rawGyroZ = 0;
    int16_t rawTemp = 0;

    float accelX = 0, accelY = 0, accelZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;
    float temperature = 0;

    float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

    float accelScale = 16384.0f;
    float gyroScale  = 131.0f;

    float pitch = 0, roll = 0, yaw = 0;

    uint32_t prevMicros = 0;

    // 互补滤波系数：越大越信任陀螺仪
    static constexpr float alpha = 0.98f;

    void writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((int)addr, 1);
        if (Wire.available()) return Wire.read();
        return 0xFF;
    }

    // 连续读取 14 字节：加速度 XYZ + 温度 + 陀螺仪 XYZ
    void readAllData() {
        Wire.beginTransmission(addr);
        Wire.write(MPU6050_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((int)addr, 14);

        if (Wire.available() == 14) {
            rawAccX  = (Wire.read() << 8) | Wire.read();
            rawAccY  = (Wire.read() << 8) | Wire.read();
            rawAccZ  = (Wire.read() << 8) | Wire.read();
            rawTemp  = (Wire.read() << 8) | Wire.read();
            rawGyroX = (Wire.read() << 8) | Wire.read();
            rawGyroY = (Wire.read() << 8) | Wire.read();
            rawGyroZ = (Wire.read() << 8) | Wire.read();
        }
    }
};

#endif
