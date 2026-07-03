#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <Wire.h>
#include <math.h>
#include "config.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

extern "C" {
#include "../DMP/inv_mpu.h"
#include "../DMP/inv_mpu_dmp_motion_driver.h"
void hal_set_mpu_addr(uint8_t addr);
}

// I2C HAL 函数声明（在 dmp_integration.cpp 中实现）
extern int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                     unsigned char length, unsigned char const *data);
extern int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                    unsigned char length, unsigned char *data);

// ---- DMP 包装类 ----
class MPU6050_DMP {
public:
    bool begin(uint8_t address = 0x68) {
        addr = address;
        hal_set_mpu_addr(address);  // 设置 HAL 层的 I2C 地址
        Wire.begin();

        delay(100);

        // 初始化 MPU
        struct int_param_s int_param;
        memset(&int_param, 0, sizeof(int_param));
        if (mpu_init(&int_param) != 0) {
            return false;
        }

        // 设置传感器
        if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
            return false;
        }

        // 设置采样率
        if (mpu_set_sample_rate(DMP_SAMPLE_RATE) != 0) {
            return false;
        }

        // 配置 FIFO
        if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
            return false;
        }

        // 加载 DMP 固件
        if (dmp_load_motion_driver_firmware() != 0) {
            return false;
        }

        // 使能四元数输出
        if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO) != 0) {
            return false;
        }

        // 设置 DMP FIFO 速率
        if (dmp_set_fifo_rate(DMP_SAMPLE_RATE) != 0) {
            return false;
        }

        // 启用 DMP
        if (mpu_set_dmp_state(1) != 0) {
            return false;
        }

        last_update_time = millis();
        return true;
    }

    void update() {
        short gyro[3], accel[3];
        long quat[4];
        unsigned long timestamp;
        short sensors;
        unsigned char more;

        // 从 DMP FIFO 读取数据
        if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0) {
            // 四元数格式: q30 (fixed point)
            // 转换为浮点并归一化
            float q0 = quat[0] / 1073741824.0f;  // 2^30
            float q1 = quat[1] / 1073741824.0f;
            float q2 = quat[2] / 1073741824.0f;
            float q3 = quat[3] / 1073741824.0f;

            // 四元数转欧拉角
            // pitch (绕 Y 轴)
            pitch = atan2(2.0f * (q0 * q2 - q1 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
            // roll (绕 X 轴)
            roll = asin(2.0f * (q0 * q1 + q2 * q3)) * 180.0f / PI;
            // yaw (绕 Z 轴)
            yaw = atan2(2.0f * (q0 * q3 - q1 * q2), 1.0f - 2.0f * (q1 * q1 + q3 * q3)) * 180.0f / PI;

            // 加速度计 (g)
            accelX = (float)accel[0] / 16384.0f;
            accelY = (float)accel[1] / 16384.0f;
            accelZ = (float)accel[2] / 16384.0f;

            // 陀螺仪 (°/s)
            gyroX = (float)gyro[0] / 131.0f;
            gyroY = (float)gyro[1] / 131.0f;
            gyroZ = (float)gyro[2] / 131.0f;

            last_update_time = millis();
        }
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

    uint8_t getAddress() const { return addr; }

    unsigned long getLastUpdateTime() const { return last_update_time; }

private:
    uint8_t addr = 0x68;

    float pitch = 0, roll = 0, yaw = 0;
    float accelX = 0, accelY = 0, accelZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;

    unsigned long last_update_time = 0;
};

#endif // MPU6050_DMP_H
