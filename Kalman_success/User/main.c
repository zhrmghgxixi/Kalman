#include "stm32f10x.h"
#include "delay.h"
#include "iic.h"
#include "mpu6050.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>                    // for snprintf
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

// 加速度计和陀螺仪换算常量
#define ACC_SENS   16384.0f       // ±2g 模式下：16384 LSB/g
#define GYRO_SENS  16.4f          // ±2000 °/s 模式下：16.4 LSB/(°/s)
#define RAD2DEG    57.29577951f   // 弧度转度

// Kalman 滤波器结构体
typedef struct {
    float Q_angle;    // 角度过程噪声协方差
    float Q_bias;     // 偏置过程噪声协方差
    float R_measure;  // 测量噪声协方差
    float angle;      // 滤波后角度
    float bias;       // 陀螺仪偏置
    float rate;       // 去偏置角速度
    float P[2][2];    // 误差协方差矩阵
} Kalman;

// Kalman 滤波器初始化
void Kalman_Init(Kalman *kf)
{
    kf->Q_angle    = 0.001f;
    kf->Q_bias     = 0.003f;
    kf->R_measure  = 0.03f;
    kf->angle      = 0.0f;
    kf->bias       = 0.0f;
    kf->rate       = 0.0f;
    kf->P[0][0]    = 0.0f;
    kf->P[0][1]    = 0.0f;
    kf->P[1][0]    = 0.0f;
    kf->P[1][1]    = 0.0f;
}

// Kalman 滤波器主函数：输出滤波后角度
float Kalman_GetAngle(Kalman *kf, float newAngle, float newRate, float dt)
{
    // 1) 预测
    kf->rate  = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    // 2) 更新误差协方差
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    // 3) 计算卡尔曼增益
    float S   = kf->P[0][0] + kf->R_measure;
    float K0  = kf->P[0][0] / S;
    float K1  = kf->P[1][0] / S;
    // 4) 更新角度和偏置
    float y   = newAngle - kf->angle;
    kf->angle += K0 * y;
    kf->bias  += K1 * y;
    // 5) 更新协方差矩阵
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[1][0] = kf->P[1][0] - K1 * P00;
    kf->P[1][1] = kf->P[1][1] - K1 * P01;
    return kf->angle;
}

// 全局变量，用于存放原始传感器值
short ax, ay, az;
short gx, gy, gz;

int main(void)
{
    // 1) 初始化硬件
    delay_init();   // SysTick
    IIC_Init();     // 软件 I2C
    OLED_Init();    // OLED
    if (MPU_Init()) {
        OLED_Clear();
        OLED_ShowString(1, 1, "MPU Init Err");
        while (1);
    }
    // 验证 WHO_AM_I
    {
        u8 id = MPU_Read_Byte(MPU_DEVICE_ID_REG);
        OLED_Clear();
        OLED_ShowString(1, 1, "WHO_AM_I=");
        OLED_ShowHexNum(1, 11, id, 2);
        delay_ms(1000);
    }

    // 2) Kalman 实例化（Roll, Pitch, Yaw）
    Kalman kalmanRoll, kalmanPitch, kalmanYaw;
    Kalman_Init(&kalmanRoll);
    Kalman_Init(&kalmanPitch);
    Kalman_Init(&kalmanYaw);

    // 滤波周期
    const float dt = 0.200f;  // 与 delay_ms(200) 对应
    static float yaw = 0.0f;

    // 3) 主循环
    while (1)
    {
        // 3.1) 读取原始数据
        MPU_Get_Accelerometer(&ax, &ay, &az);
        MPU_Get_Gyroscope(&gx, &gy, &gz);

        // 3.2) 换算到物理量
        float ax_g   = (float)ax / ACC_SENS;
        float ay_g   = (float)ay / ACC_SENS;
        float az_g   = (float)az / ACC_SENS;
        float gx_dps = (float)gx / GYRO_SENS;
        float gy_dps = (float)gy / GYRO_SENS;
        float gz_dps = (float)gz / GYRO_SENS;

        // 3.3) 加速度测角
        float rollAcc  = atan2f(ay_g, az_g) * RAD2DEG;
        float pitchAcc = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * RAD2DEG;

        // 3.4) 卡尔曼滤波计算 Roll & Pitch
        float roll  = Kalman_GetAngle(&kalmanRoll,  rollAcc,  gx_dps, dt);
        float pitch = Kalman_GetAngle(&kalmanPitch, pitchAcc, gy_dps, dt);

        // 3.5) Yaw：先积分，再卡尔曼滤波平滑
        float yawGyro = yaw + gz_dps * dt;
        yaw = Kalman_GetAngle(&kalmanYaw, yawGyro, gz_dps, dt);

        // 4) 显示结果
        OLED_Clear();
        OLED_ShowString(1, 1, "R:"); OLED_ShowSignedNum(1, 4, (int)roll,  5);
        OLED_ShowString(2, 1, "P:"); OLED_ShowSignedNum(2, 4, (int)pitch, 5);
        OLED_ShowString(3, 1, "Y:"); OLED_ShowSignedNum(3, 4, (int)yaw,   5);

        // 5) 等待下一个周期
        delay_ms((uint16_t)(dt * 1000));
    }
}
