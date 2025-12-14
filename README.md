# Reusable Autonomous VTOL Vehicle — Guidance, Navigation, and Control (GNC)

This repository contains the embedded firmware and supporting Python tools for Guidance, Navigation, and Control of a VTOL vehicle/rocket to follow a predetermined trajectory. It fuses IMU (MPU9250), GPS (u-blox), and barometer (BMP280) measurements using an attitude EKF (AHRS_EKF) and a position EKF (POS_EKF), and optionally streams telemetry over NRF24L01 for visualization.

https://github.com/user-attachments/assets/915736a8-9bbe-4e83-aeb2-1a3086ffe436

## Overview
- Purpose: Real-time estimation of attitude (quaternion), velocity, and position in NED frame, suitable for autonomous VTOL/rocket trajectory tracking.
- Sensors: MPU9250 IMU (accel/gyro/mag), u-blox GPS (NAV-PVT UBX), BMP280 barometer.
- Filters:
  - AHRS_EKF: Quaternion-based EKF combining gyro (process) with accel+mag (measurement).
  - POS_EKF: 9-state EKF estimating acceleration, velocity, and position in NED frame, driven by IMU and corrected by GPS/BMP.
- Telemetry: Optional NRF24L01 (RF24) link to transmit 8 floats per packet.
- Visualization: Python scripts for attitude (Panda3D) and trajectory (Matplotlib).

## Repository Structure

- `README.md` — This document.
- `Documents/Arduino/libraries/major_project/`
  - `MPU9250.h/.cpp` — Bolder Flight Systems MPU9250 driver (GPL-3.0). Provides IMU readings.
  - `ahrs_ekf.h/.cpp` — Quaternion attitude EKF. Predicts with gyro; corrects with accel+mag.
  - `pos_ekf.h/.cpp` — Position EKF in NED frame. Inputs: quaternion+accel; Measurements: GPS vel + LLA + BMP altitude.
  - `mpu_pose_ekf.h/.cpp` — Glue module orchestrating sensor setup, bias compensation, AHRS and POS updates.
  - `gps.h/.cpp` — u-blox UBX NAV-PVT parser over `Serial1`. Exposes iTOW, NED velocities, LLA, MSL altitude.
  - `bmp.h/.cpp` — Adafruit BMP280 wrapper. Exposes altitude in meters via `bmp_altitude`.
  - `nrf.h/.cpp` — NRF24L01 telemetry (RF24). Sends/receives 8-float `Data_Package`.
  - `python_ahrs/`
    - `ekf_ahrs.py` — Panda3D visualization of attitude (reads quaternion from serial).
    - `position.py` — Matplotlib animated 3D plot of trajectory (reads N, E, D from serial).
    - `config/config.prc` — Panda3D configuration file.
    - `run_ekf_ahrs.bat`, `run_position.bat` — Convenience launchers (adjust COM port as needed).
  - `teensy_examples/ahrs_quat/ahrs_quat.ino` — Example sketch printing quaternion; note include header comment.
  - `readme.txt` — Legacy notes.

## Data Flow
1. IMU sampling (`MPU9250`): accel/gyro/mag at configured rates (e.g. Ts = 0.004 s).
2. Bias compensation: averages over startup to subtract accel and gyro biases; magnetometer ellipse compensation.
3. AHRS_EKF: produces normalized quaternion `q` from accel, mag, gyro.
4. GPS/BMP: UBX NAV-PVT at 10 Hz; BMP altitude sampled when new GPS data arrives; BMP vertical velocity estimated from altitude delta.
5. POS_EKF: inputs `[q0 q1 q2 q3 ax ay az]`, corrects when iTOW increments; converts LLA to NED relative to a startup reference.
6. Telemetry: optional NRF24L01 sends up to 8 floats (e.g., quaternion and NED components).
7. Visualization: Python scripts display attitude cube or 3D trajectory.

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
- Example sketch `ahrs_quat.ino` includes `mpu_ahrs_ekf.h` which is not present; replace with `mpu_pose_ekf.h` or your wrapper.
- In `gps_setup()`, the line `while(!processGPS);` should be `while(!processGPS());` to actually call the function.
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
