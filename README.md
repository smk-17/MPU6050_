# MPU6050_
# MPU6050 IMU with DMA and Kalman Filter — STM32

A simple embedded project using an **MPU6050 IMU** with an **STM32F401RE** microcontroller. The project reads accelerometer and gyroscope data over **I2C**, uses **DMA** for sensor data acquisition, and applies a **Kalman filter** to estimate the Roll and Pitch angles.

## Features

* MPU6050 communication using I2C
* MPU6050 register initialization and raw data reading
* Accelerometer and gyroscope data conversion
* I2C RX using DMA
* DMA completion and I2C error callbacks
* Roll and Pitch calculation from accelerometer data
* Roll and Pitch estimation using a 2-state Kalman filter
* Gyroscope-based yaw integration
* UART output for monitoring sensor and filter data
* ITM/SWV output support through `printf`

## Hardware

* **STM32F401RE**
* **MPU6050 6-axis IMU**
* USB/UART connection for serial output

## Software

* STM32CubeIDE
* STM32 HAL
* Embedded C
* I2C
* DMA
* UART
* Kalman Filter

## How It Works

The MPU6050 is connected to the STM32 through I2C.

The program first verifies the sensor using the `WHO_AM_I` register and configures the MPU6050 for:

* ±2g accelerometer range
* ±250 °/s gyroscope range
* Configured sample-rate divider and digital low-pass filter

The sensor's 14-byte measurement block is then transferred using **I2C DMA**. Once the transfer is complete, the DMA callback signals that new data is available.

The raw values are converted into:

* Accelerometer → `g`
* Gyroscope → `degrees/second`

Accelerometer measurements are used to calculate Roll and Pitch angles:

```text
Roll  = atan2(AccY, AccZ)
Pitch = atan2(-AccX, sqrt(AccY² + AccZ²))
```

The accelerometer angles are then combined with gyroscope rate measurements using a **2-state Kalman filter**. The filter estimates both:

* Angle
* Gyroscope bias

Yaw is currently obtained by integrating the Z-axis gyroscope rate over time.

## Data Flow

```text
             MPU6050
                │
                │ I2C
                ▼
          STM32F401RE
                │
          I2C + DMA RX
                │
                ▼
        Raw Sensor Data
                │
                ▼
        Unit Conversion
                │
        ┌───────┴────────┐
        ▼                ▼
 Accelerometer        Gyroscope
        │                │
        ▼                ▼
  Roll/Pitch Angle     Angular Rate
        │                │
        └───────┬────────┘
                ▼
          Kalman Filter
                │
                ▼
        Roll / Pitch Estimate
```

## Project Structure

```text
MPU-6050/
├── Core/
│   ├── Inc/
│   │   ├── MPU6050.h
│   │   ├── Kalman.h
│   │   └── main.h
│   │
│   └── Src/
│       ├── MPU6050.c
│       ├── Kalman.c
│       └── main.c
│
├── Drivers/
├── MPU_6050.ioc
└── README.md
```

### Main Modules

**`MPU6050.c / MPU6050.h`**

Handles MPU6050 initialization, register access, DMA-based data acquisition, raw data extraction, and unit conversion.

**`Kalman.c / Kalman.h`**

Implements the 2-state Kalman filter used for Roll and Pitch estimation.

**`main.c`**

Initializes the peripherals, starts DMA acquisition, processes sensor data, calculates orientation, and prints the results.

## Output

The program outputs sensor and orientation information through UART, for example:

```text
ACC: X=0.02 Y=-0.01 Z=1.00 g

GYRO: X=0.15 Y=-0.20 Z=0.40 dps

Roll: ACC=-0.57 | Pitch: ACC=-1.15

Roll=-0.32 Pitch=-0.81 Yaw=2.14
```


## Author

**Srinivas Kumar**

MPU 6050 - DeviceDriver / STM32 
