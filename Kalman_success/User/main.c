#include "stm32f10x.h"
#include "delay.h"
#include "iic.h"
#include "mpu6050.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>                    // for snprintf
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

// ���ٶȼƺ������ǻ��㳣��
#define ACC_SENS   16384.0f       // ��2g ģʽ�£�16384 LSB/g
#define GYRO_SENS  16.4f          // ��2000 ��/s ģʽ�£�16.4 LSB/(��/s)
#define RAD2DEG    57.29577951f   // ����ת��

// Kalman �˲����ṹ��
typedef struct {
    float Q_angle;    // �Ƕȹ�������Э����
    float Q_bias;     // ƫ�ù�������Э����
    float R_measure;  // ��������Э����
    float angle;      // �˲���Ƕ�
    float bias;       // ������ƫ��
    float rate;       // ȥƫ�ý��ٶ�
    float P[2][2];    // ���Э�������
} Kalman;

// Kalman �˲�����ʼ��
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

// Kalman �˲���������������˲���Ƕ�
float Kalman_GetAngle(Kalman *kf, float newAngle, float newRate, float dt)
{
    // 1) Ԥ��
    kf->rate  = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    // 2) �������Э����
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    // 3) ���㿨��������
    float S   = kf->P[0][0] + kf->R_measure;
    float K0  = kf->P[0][0] / S;
    float K1  = kf->P[1][0] / S;
    // 4) ���½ǶȺ�ƫ��
    float y   = newAngle - kf->angle;
    kf->angle += K0 * y;
    kf->bias  += K1 * y;
    // 5) ����Э�������
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[1][0] = kf->P[1][0] - K1 * P00;
    kf->P[1][1] = kf->P[1][1] - K1 * P01;
    return kf->angle;
}

// ȫ�ֱ��������ڴ��ԭʼ������ֵ
short ax, ay, az;
short gx, gy, gz;

int main(void)
{
    // 1) ��ʼ��Ӳ��
    delay_init();   // SysTick
    IIC_Init();     // ��� I2C
    OLED_Init();    // OLED
    if (MPU_Init()) {
        OLED_Clear();
        OLED_ShowString(1, 1, "MPU Init Err");
        while (1);
    }
    // ��֤ WHO_AM_I
    {
        u8 id = MPU_Read_Byte(MPU_DEVICE_ID_REG);
        OLED_Clear();
        OLED_ShowString(1, 1, "WHO_AM_I=");
        OLED_ShowHexNum(1, 11, id, 2);
        delay_ms(1000);
    }

    // 2) Kalman ʵ������Roll, Pitch, Yaw��
    Kalman kalmanRoll, kalmanPitch, kalmanYaw;
    Kalman_Init(&kalmanRoll);
    Kalman_Init(&kalmanPitch);
    Kalman_Init(&kalmanYaw);

    // �˲�����
    const float dt = 0.200f;  // �� delay_ms(200) ��Ӧ
    static float yaw = 0.0f;

    // 3) ��ѭ��
    while (1)
    {
        // 3.1) ��ȡԭʼ����
        MPU_Get_Accelerometer(&ax, &ay, &az);
        MPU_Get_Gyroscope(&gx, &gy, &gz);

        // 3.2) ���㵽������
        float ax_g   = (float)ax / ACC_SENS;
        float ay_g   = (float)ay / ACC_SENS;
        float az_g   = (float)az / ACC_SENS;
        float gx_dps = (float)gx / GYRO_SENS;
        float gy_dps = (float)gy / GYRO_SENS;
        float gz_dps = (float)gz / GYRO_SENS;

        // 3.3) ���ٶȲ��
        float rollAcc  = atan2f(ay_g, az_g) * RAD2DEG;
        float pitchAcc = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * RAD2DEG;

        // 3.4) �������˲����� Roll & Pitch
        float roll  = Kalman_GetAngle(&kalmanRoll,  rollAcc,  gx_dps, dt);
        float pitch = Kalman_GetAngle(&kalmanPitch, pitchAcc, gy_dps, dt);

        // 3.5) Yaw���Ȼ��֣��ٿ������˲�ƽ��
        float yawGyro = yaw + gz_dps * dt;
        yaw = Kalman_GetAngle(&kalmanYaw, yawGyro, gz_dps, dt);

        // 4) ��ʾ���
        OLED_Clear();
        OLED_ShowString(1, 1, "R:"); OLED_ShowSignedNum(1, 4, (int)roll,  5);
        OLED_ShowString(2, 1, "P:"); OLED_ShowSignedNum(2, 4, (int)pitch, 5);
        OLED_ShowString(3, 1, "Y:"); OLED_ShowSignedNum(3, 4, (int)yaw,   5);

        // 5) �ȴ���һ������
        delay_ms((uint16_t)(dt * 1000));
    }
}
