

---

# Kalman Filter with STM32F103C8T6

## Project Overview

This project implements a Kalman filter to process sensor data and estimate angles, specifically using an STM32F103C8T6 microcontroller. The project utilizes STM32's low-level peripheral libraries for I2C communication and sensor data acquisition.

### Key Features

* **Kalman Filter**: A widely used algorithm for sensor fusion, combining accelerometer and gyroscope data to estimate angles with reduced noise.
* **STM32F103C8T6**: The project uses the STM32F103C8T6 microcontroller, supported by STM32's HAL/LL libraries for hardware abstraction.
* **Sensor Integration**: The system works with accelerometer and gyroscope sensors to gather data for the Kalman filter.

---

## Libraries Used

### 1. STM32F103C8T6 Low-Level Libraries

These are the core libraries for STM32 peripherals, providing access to low-level hardware features like I2C for communication with sensors.

### 2. Kalman Filter Algorithm

This project includes a **Kalman filter** algorithm that combines the data from accelerometers and gyroscopes to accurately estimate the angle. The filter uses predictions, updates, and corrections based on the incoming data.

```c
// Kalman Filter Update Function
float Kalman_GetAngle(Kalman *kf, float newAngle, float newRate, float dt) {
    // Prediction step
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update the covariance matrix
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[0][1] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Calculate the Kalman gain
    float S = kf->P[0][0] + kf->R_measure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    // Update the angle
    float y = newAngle - kf->angle;
    kf->angle += K0 * y;
    kf->bias += K1 * y;

    // Update the covariance matrix
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
}
```

This Kalman filter is designed to minimize the noise from sensor data and provides an optimal estimate of the angle over time.

---

## Setup Instructions

1. **Install STM32CubeMX**:

   * Use STM32CubeMX to configure the microcontroller, peripherals, and generate initialization code.

2. **Initialize STM32 Project**:

   * Set up a Keil project or any IDE of your choice for STM32 development.

3. **Include Necessary Libraries**:

   * Include the STM32F1xx standard peripheral libraries or HAL/LL libraries for peripheral management.
   * Link to the appropriate files for Kalman filter and sensor libraries.

4. **Sensor Integration**:

   * Interface with accelerometer and gyroscope sensors via I2C for data collection.

5. **Compile and Flash**:

   * Compile the code and flash it to the STM32F103C8T6.

---

## System Architecture

* **STM32F103C8T6**: The microcontroller that acts as the central processing unit.
* **I2C Communication**: Used for interfacing with sensors.
* **Kalman Filter**: Processes sensor data (accelerometer and gyroscope) to estimate angles.
* **Sensor Data**: The system collects sensor data and feeds it to the Kalman filter for processing.

---

## Contributing

Feel free to fork this repository, make changes, and submit pull requests.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

### 结尾说明：
