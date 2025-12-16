# Reusable Autonomous VTOL Vehicle — Guidance, Navigation, and Control (GNC)

This repository contains the embedded firmware and supporting Python tools for Guidance, Navigation, and Control of a VTOL vehicle/rocket to follow a predetermined trajectory. It fuses IMU (MPU9250), GPS (u-blox), and barometer (BMP280) measurements using an attitude EKF (AHRS_EKF) and a position EKF (POS_EKF), and optionally streams telemetry over NRF24L01 for visualization.

https://github.com/user-attachments/assets/915736a8-9bbe-4e83-aeb2-1a3086ffe436

## Overview
- Purpose: Keeps track of which way the drone is pointing, how fast it’s moving, and where it is—live—so it can follow a pre-planned flight path.  
- Sensors: A motion chip (MPU9250) feels bumps and spins, a GPS (u-blox) tells latitude/longitude, and a tiny barometer (BMP280) reads height.  
- Smart filters:  
  – AHRS_EKF: blends gyro, accelerometer, and compass to give a steady “which-way-is-up” quaternion.  
  – POS_EKF: mixes IMU data with GPS and barometer to deliver clean acceleration, speed, and position.  
- Radio (optional): NRF24L01 sends a quick 8-number snapshot to the ground.  
- Laptop view: Python plots a 3-D cube showing attitude and a map showing the flown path.

## What’s Inside

- **README.md** – the file you’re reading now.  
- **Documents/Arduino/libraries/major_project/** – the main code folder.  
  - **MPU9250** – talks to the motion sensor (accelerometer, gyro, compass).  
  - **ahrs_ekf** – figures out which way the vehicle is pointing.  
  - **pos_ekf** – works out speed and position.  
  - **mpu_pose_ekf** – brings the two filters above together and keeps them running smoothly.  
  - **gps** – reads location and speed from the GPS module.  
  - **bmp** – reads height from the barometer.  
  - **nrf** – optional radio link that can send data to the ground.  
  - **python_ahrs/** – handy Python programs that draw a 3-D picture of the flight on your computer.  
  - **teensy_examples/** – a small ready-to-run example you can upload to the board.  
  - **readme.txt** – old notes (can be ignored).

## Data Flow

### IMU Sampling (MPU9250)
- The system samples an MPU9250 IMU at a fixed rate (example: 250 Hz, `Ts = 0.004 s`).
- Accelerometer, gyroscope, and magnetometer data are acquired at each sample.

### Bias & Sensor Compensation
- At startup, bias calibration is performed by averaging initial samples.
- Accelerometer and gyroscope offsets are estimated and subtracted.
- Magnetometer measurements are corrected using ellipse (hard/soft iron) compensation.

### AHRS EKF
- An Attitude and Heading Reference System based on an Extended Kalman Filter (EKF) is used.
- Accelerometer, gyroscope, and magnetometer data are fused to estimate orientation.
- The filter outputs a normalized quaternion `q`.

### GPS & Barometer (BMP)
- GPS data is received using UBX NAV-PVT messages at 10 Hz.
- GPS provides position and time information.
- A BMP barometer provides altitude measurements.
- Barometric altitude is sampled only when new GPS data arrives.
- Vertical velocity is computed from successive barometric altitude differences.

### Position EKF (POS_EKF)
- Inputs include orientation `q` and body-frame accelerations `[ax, ay, az]`.
- EKF corrections are applied when the GPS `iTOW` value changes.
- GPS latitude, longitude, and altitude (LLA) are converted to local NED coordinates.
- The NED frame is defined relative to a startup reference point.

### Telemetry
- Telemetry can be transmitted via an NRF24L01 radio link.
- The link is limited to 8 floating-point values.
- Typical telemetry includes quaternion and NED position/velocity.

### Visualization
- Python scripts are used for visualization.
- Orientation is displayed using an attitude cube.
- Motion is visualized as a 3D flight or trajectory plot.


## Hardware
- Microcontroller: Teensy (tested) or Arduino compatible with `Serial1`, I2C, SPI.
- Sensors:
  - `MPU9250` IMU (I2C default address `0x68`).
  - `u-blox` GPS (UART on `Serial1`).
  - `BMP280` Barometer (I2C).
- Radio (optional): `NRF24L01` with CE=9, CSN=10 (adjust pins as needed).

## Software Requirements
- Arduino/Teensy toolchain and board support.
- Libraries:
  - `Adafruit_BMP280` (barometer).
  - `RF24` and `nRF24L01` (radio).
  - `Wire`/`SPI` (core).
  - Eigen for Arduino (`eigen.h` shim) included by project; ensure headers are available to your environment.
- Python (optional visualization):
  - `ahrs`, `panda3d`, `pyserial`, `matplotlib`, `numpy`.

## Setup and Usage
### Arduino library installation
- Place `major_project` under your Arduino libraries path (already organized as a library directory).
- Include headers from your sketch as needed, or build/test using the `teensy_examples` sketch.

### Sensor initialization
- Call `pose_setup()` once:
  - Internally calls `imu_setup()` to configure the IMU ranges and bandwidth.
  - Reads IMU to estimate initial quaternion via accel+mag average (`init_quaternion`).
  - Runs `gps_setup()`/`bmp_setup()`; computes LLA reference by averaging initial GPS/BMP readings.
- In your main loop, call `pose_update()` to:
  - Update `q` via AHRS_EKF.
  - Update POS_EKF with IMU input; correct using GPS/BMP when a new iTOW is detected.

### Output variables
- `q` (double[4]): attitude quaternion [w, x, y, z].
- `x` (double[9]): [aN, aE, aD, vN, vE, vD, pN, pE, pD] in NED, meters and m/s.

### Python visualizations
- Edit `python_ahrs/ekf_ahrs.py` or `position.py` to set your serial port (default `COM10`) and baud `2000000`.
- Launch with `python python_ahrs/ekf_ahrs.py` or `python python_ahrs/position.py`, or use the `.bat` files.
- Expected serial format per line (comma-separated 8 columns):
  - columns[1..4] = quaternion (w, x, y, z)
  - columns[5..7] = N, E, D (meters)

## Configuration Notes
- Sea-level pressure in `bmp_read()` is set to `1019.66 hPa`; adjust for local SLP to improve altitude accuracy.
- GPS UBX rate configured to 10 Hz; see `UBLOX_INIT` for options.
- Frames: IMU uses body axes; POS_EKF outputs NED.
- Units: accel m/s², velocity m/s, position meters, angles radians (LLA converted to radians in POS_EKF).

## Known Issues and Caveats
- Magnetometer calibration constants are hard-coded; consider re-calibrating for your hardware.
- Ensure `Serial1` pins, I2C/SPI wiring, and RF24 CE/CSN pins match your board.

## Contributing
- Open issues and pull requests are welcome. Please include hardware setup, logs, and steps to reproduce.
- For new sensors or frames, maintain consistent units and document your changes.

## License
-MIT License. See LICENSE.

## Acknowledgements
- MPU9250 driver by Bolder Flight Systems (GPL-3.0).
- Python AHRS library (https://ahrs.readthedocs.io/).

## Citation
If you use this code in academic work, please cite and link to this repository and acknowledge sensor and library authors.
