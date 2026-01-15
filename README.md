# Reusable Autonomous VTOL Vehicle — Guidance, Navigation, and Control (GNC)

A comprehensive embedded firmware and Python toolset for Guidance, Navigation, and Control (GNC) of a VTOL vehicle/rocket enabling precise trajectory following through active control systems. This project implements advanced sensor fusion using IMU (MPU9250), GPS (u-blox), barometric altitude (BMP280), and Lidar measurements with Extended Kalman Filters to achieve soft and accurate landings on predetermined trajectories.

https://github.com/user-attachments/assets/915736a8-9bbe-4e83-aeb2-1a3086ffe436

## Executive Summary

### Motivation & Background
In interplanetary missions, the landing of space vehicles is typically accomplished using parachutes. However, this simple method faces significant challenges—vehicles are prone to parachute drifts that are difficult to predict, especially on planets with dense atmospheres. Recent efforts have focused on developing active control systems for space vehicles, enabling precise guidance, navigation, and control over predetermined trajectories for soft and accurate landings.

This project aims to implement GNC control algorithms on an Electric Ducted Fan (EDF) powered model of a VTOL vehicle, enabling it to follow a fixed trajectory. By utilizing these advanced control systems, space vehicles can navigate more accurately and efficiently, reducing risks and costs associated with interplanetary travel while increasing vehicle reusability and payload delivery efficiency.

### Key Objectives
- Develop and implement advanced guidance, navigation, and control algorithms for precise trajectory following
- Fuse multi-sensor data (IMU, GPS, barometer, Lidar) for robust state estimation
- Implement attitude determination using extended Kalman filters and quaternion representations
- Design and validate control systems (PID, LQG, MPC) for trajectory control
- Enable soft and precise landing on planetary surfaces through active control

### Applications
- **Interplanetary missions**: Precise soft landings overcoming parachute drift challenges
- **Payload delivery systems**: Efficient and cost-effective delivery platforms
- **UAVs and drones**: Enhanced autonomous navigation and control
- **Defense applications**: Military drone navigation and precision landing
- **Commercial applications**: Autonomous delivery, inspection services, and aerial operations

---

## System Architecture

### Hardware Components

#### Microcontroller Options
The system supports multiple microcontroller platforms with different performance characteristics:

**Teensy 4.1** (Recommended)
- Clock frequency: 600 MHz
- Program memory: 2 MB Flash
- Digital pins: 34
- Analog pins: 18
- PWM pins: 22
- Hardware serial ports: 8
- Interrupt pins: 26
- Voltage range: 3.3V to 6.0V

**ATmega328P** (Low-power alternative)
- Clock frequency: 20 MHz
- Program memory: 32 KB Flash
- Digital pins: 14
- Analog pins: 6
- PWM pins: 6
- Hardware serial ports: 1
- Interrupt pins: 2
- Voltage range: 1.8V to 5.5V

The Teensy 4.1 offers significantly higher performance for real-time control applications, while the ATmega328P provides a low-cost, low-power option for simpler applications.

#### Sensor Suite

**MPU9250: 9-Axis Inertial Measurement Unit**
- 3-axis gyroscope: ±250°/s to ±2000°/s (programmable)
- 3-axis accelerometer: ±2g to ±16g (programmable)
- 3-axis magnetometer: ±4800 μT
- Digital Low Pass Filter with programmable bandwidth (5 Hz to 260 Hz)
- Internal Digital Motion Processor (DMP) with 1000 Hz max output rate
- I2C interface (default address: 0x68)

**BMP280: Barometric Pressure & Temperature Sensor**
- Pressure measurement: ±1 hPa accuracy
- Temperature measurement: ±1°C accuracy
- Operating range: -40°C to 85°C
- Altitude resolution: ~0.5 m
- I2C and SPI interfaces
- 16-bit ADC converter

**NEO-M8N GPS Module**
- 72-channel GPS receiver
- Multi-constellation support: GPS, GLONASS, Galileo, BeiDou
- Positional accuracy: up to 2.5 meters
- Protocol support: NMEA, UBX, RTCM
- Built-in data logger (16 hours storage)
- UART interface at configurable baud rates

**TFmini Lidar: Laser Distance Measurement**
- Measurement range: up to 12 meters
- Resolution: 5 mm
- Field of view: 2.3 degrees
- UART interface
- Ideal for precision ground-relative altitude during landing

**NRF24L01: 2.4 GHz Transceiver**
- ISM band operation: 2.4 GHz
- Range: up to 100 meters (open space)
- Data rate: up to 2 Mbps
- SPI interface (CE=9, CSN=10)
- Features: Auto-retransmission, dynamic channel selection, multi-level data pipes

#### Power Management
- Primary battery: 6S 65C 5000 mAh LiPo (nominal 22.2V)
- LM2596 buck converter: 22V → 5V regulation
- AMS1117 regulator: 5V → 3.3V for microcontroller and sensors
- Stable, regulated power distribution to all components

#### Actuation System
- **Electric Ducted Fan (EDF)**: Primary thrust source
- **Thrust Vectoring Control (TVC)**: 
  - Nozzle-based vector control for thrust direction adjustment
  - Vane-based control for aerodynamic vectoring
  - Servo actuators for angular positioning

---

## Theoretical Foundations

### Sensor Fusion & State Estimation

#### Extended Kalman Filter (EKF) Theory

The Extended Kalman Filter is a nonlinear state estimation algorithm that fuses multi-sensor measurements to estimate the state of a dynamic system. The state vector includes position, velocity, and orientation:

$$\mathbf{x}_k = [p_k, v_k, q_k]^T$$

where $p_k$ is position, $v_k$ is velocity, and $q_k$ is a unit quaternion representing attitude.

**Prediction Step:**
$$\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_k, \Delta t)$$

**Update Step:**
$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k(\mathbf{z}_k - h_k(\hat{\mathbf{x}}_k^-))$$

**Kalman Gain:**
$$K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R_k)^{-1}$$

**Covariance Update:**
$$P_k = (I - K_k H_k) P_k^-$$

where:
- $P_k^-$ = predicted state covariance
- $H_k$ = measurement Jacobian matrix
- $R_k$ = measurement noise covariance
- $\mathbf{z}_k$ = sensor measurements

**Advantages of EKF:**
- Handles nonlinear system dynamics and measurements
- Optimal for linear systems, near-optimal for slightly nonlinear systems
- Computationally efficient for embedded systems
- Well-established and robust implementation
- Asynchronous processing capability for multiple sensor rates

#### AHRS (Attitude and Heading Reference System) - EKF Implementation

The AHRS_EKF fuses accelerometer, gyroscope, and magnetometer data to estimate vehicle orientation as a unit quaternion $q = [w, x, y, z]^T$.

**Measurement Models:**

From accelerometer (gravity vector):
$$\mathbf{a}_{measured} = q^* \otimes [0, 0, 0, g]^T \otimes q + \mathbf{n}_a$$

From magnetometer (magnetic north):
$$\mathbf{m}_{measured} = q^* \otimes [0, \mathbf{B}]^T \otimes q + \mathbf{n}_m$$

**Gyroscope Integration:**
$$q_{k+1} = q_k \otimes \exp\left(\frac{\Delta t}{2} \boldsymbol{\omega}_k\right)$$

**Compensation Techniques:**
- Hard iron correction: $\mathbf{m}_{corrected} = C_m(\mathbf{m}_{raw} - \mathbf{b}_m)$
- Accelerometer bias estimation and removal
- Gyroscope bias tracking during initialization

#### Position EKF

The position filter estimates NED (North-East-Down) frame coordinates and velocities:

$$\mathbf{x}_{pos} = [p_N, p_E, p_D, v_N, v_E, v_D, a_N, a_E, a_D]^T$$

**State Propagation:**
- Position from velocity: $p_{k+1} = p_k + v_k \Delta t$
- Velocity from acceleration: $v_{k+1} = v_k + a_k \Delta t$
- Acceleration from IMU (in body frame, rotated to NED via attitude quaternion)

**Measurement Updates:**
- GPS position (when available at 10 Hz)
- Barometric altitude (fused with GPS altitude)
- Lidar altitude when altitude < 12 m
- Velocity estimates from successive GPS positions

#### Madgwick Filter (Complementary Alternative)

For lower computational overhead, the Madgwick filter uses gradient descent optimization:

$$q_{k+1} = q_k \otimes \exp\left(\frac{\Delta t}{2} \boldsymbol{\omega}_{bias}\right) \otimes \exp\left(\frac{\Delta t}{2} \boldsymbol{\omega}_k\right)$$

**Gyroscope Bias Update:**
$$\boldsymbol{\omega}_{bias} = \boldsymbol{\omega}_{bias} + \beta \sum_{i=1}^{n} J_{ij} F_i$$

where $\beta$ is the algorithm gain and $F_i$ is the error vector from sensor $i$.

**Magnetic Distortion Correction:**
$$\mathbf{m}_k = C_m(\mathbf{m}_{raw} - \mathbf{b}_m)$$

### System Modeling

#### First Principles Approach

Dynamic models derived from Newton's laws and rotational dynamics:

**Translational Dynamics (body frame):**
$$m\ddot{\mathbf{r}} = \mathbf{F}_{thrust} + \mathbf{F}_{gravity} + \mathbf{F}_{drag}$$

**Rotational Dynamics:**
$$I \boldsymbol{\alpha} = \boldsymbol{\tau}_{thrust} + \boldsymbol{\tau}_{drag}$$

where $I$ is the inertia matrix, $\boldsymbol{\alpha}$ is angular acceleration, and $\boldsymbol{\tau}$ are torques.

**Limitations:**
- Simplistic models often miss important nonlinear effects
- Aerodynamic coefficients difficult to determine a priori
- Cross-coupling effects between axes

#### Data-Driven System Identification

Black-box and gray-box modeling approaches for identifying transfer functions and state-space models from input-output data.

**ARX (AutoRegressive eXogenous) Model:**

Transfer function representation:
$$G(s) = \frac{b_0 + b_1 s^{-1} + \ldots + b_m s^{-m}}{a_0 + a_1 s^{-1} + \ldots + a_n s^{-n}}$$

Discrete-time form:
$$y(t) = b_0 u(t) + b_1 u(t-1) + \ldots + b_m u(t-m) - a_1 y(t-1) - \ldots - a_n y(t-n)$$

Estimation via least squares:
$$\min \sum_{i=1}^{N} [y(i) - \hat{y}(i)]^2$$

**ARMAX (AutoRegressive Moving Average with eXogenous inputs):**

Includes noise model:
$$A(q)y(t) = B(q)u(t-d) + C(q)e(t)$$

where $e(t)$ is white noise and $A$, $B$, $C$ are lag polynomials.

**Subspace Method (MOESP) for MIMO Systems:**

State-space model from block Hankel matrix decomposition:
$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$
$$\mathbf{y} = C\mathbf{x} + D\mathbf{u}$$

Steps:
1. Construct block Hankel matrix from input-output data
2. Apply Singular Value Decomposition (SVD)
3. Extract state and output subspaces
4. Solve least-squares optimization to recover system matrices

**With Noise Model:**
$$\min_{D,G} \left\| Y - D \cdot U - G \cdot N \right\|_F^2$$

subject to $X^T X = I$

#### Time-Invariance Property

A system is time-invariant if its response to input is independent of when the input is applied. For linear systems, impulse response remains constant across all time intervals. This property is crucial for the validity of transfer function and state-space models.

### Control Theory & Implementation

#### PID (Proportional-Integral-Derivative) Control

The standard PID law:
$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

**Discrete form:**
$$u(k) = K_p e(k) + K_i \sum_{j=0}^{k} e(j) \Delta t + K_d \frac{e(k) - e(k-1)}{\Delta t}$$

**Tuning Methods:**
- Ziegler-Nichols method for oscillation-based tuning
- Cohen-Coon method for step response identification
- Automatic tuning via relay-feedback

**Advantages:**
- Simple and robust
- Easy to implement in real-time
- Good for linear, well-modeled systems

**Limitations:**
- No consideration of future behavior
- Limited performance in presence of constraints
- Requires separate tuning for different operating conditions

#### Linear Quadratic Regulator (LQR)

Optimal state feedback that minimizes a quadratic cost function:

$$J = \int_0^\infty \left( \mathbf{x}^T Q \mathbf{x} + \mathbf{u}^T R \mathbf{u} \right) dt$$

**Solution:**
$$\mathbf{u} = -K \mathbf{x} = -(R^{-1} B^T P) \mathbf{x}$$

where $P$ solves the algebraic Riccati equation:

$$P A + A^T P - PBR^{-1}B^T P + Q = 0$$

**Properties:**
- Provably stable closed-loop system
- Optimal in sense of minimizing cost function
- Handles multi-input, multi-output systems naturally
- Computationally efficient

**LQG (Linear Quadratic Gaussian) Variant:**
Combines LQR with Kalman filter state estimation for stochastic systems:
- LQR design for state feedback
- Kalman filter for state estimation from noisy measurements
- Separation principle guarantees closed-loop stability

#### Model Predictive Control (MPC)

MPC uses a model of the system to predict future behavior and optimize control inputs over a prediction horizon:

**Optimal Control Problem (OCP):**
$$\min_{\mathbf{u}} J_N(\mathbf{x}_0, \mathbf{u}) = \sum_{k=0}^{N-1} l(\mathbf{x}_u(k), \mathbf{u}(k))$$

subject to:
$$\mathbf{x}_{u}(k+1) = f(\mathbf{x}_u(k), \mathbf{u}(k))$$
$$\mathbf{x}_u(0) = \mathbf{x}_0$$
$$\mathbf{u}(k) \in U, \quad \forall k \in [0, N-1]$$
$$\mathbf{x}_u(k) \in X, \quad \forall k \in [0, N]$$

**Running Cost (Stage Cost):**
$$l(\mathbf{x}, \mathbf{u}) = \|\mathbf{x}_u - \mathbf{x}_r\|_Q^2 + \|\mathbf{u} - \mathbf{u}_r\|_R^2$$

**Value Function:**
$$V_N(\mathbf{x}) = \min_{\mathbf{u}} J_N(\mathbf{x}_0, \mathbf{u})$$

**Nonlinear Programming (NLP) Conversion:**
$$\min_\mathbf{w} \phi(\mathbf{w}) \quad \text{(Objective)}$$
$$g_1(\mathbf{w}) \leq 0 \quad \text{(Inequality Constraints)}$$
$$g_2(\mathbf{w}) = 0 \quad \text{(Equality Constraints)}$$

**Shooting Methods:**

*Single Shooting:*
- Discretize control inputs only
- Integrate dynamics once over entire horizon
- Faster convergence but numerically sensitive for long horizons
- Disadvantage: Nonlinearity accumulates for large N

*Multiple Shooting:*
- Discretize both controls and states
- Impose continuity constraints between intervals
- More robust for long horizons and nonlinear systems
- Better convergence properties

**Solver: IPOPT (Interior Point OPTimizer)**
- Open-source nonlinear optimization package
- Handles large-scale problems with 10,000s of variables
- Interior point algorithm with barrier functions
- Highly customizable for different problem types
- C++ implementation with interfaces to MATLAB, Python

**Implementation Framework: CasADi**
- Symbolic differentiation for accurate gradients
- Interfaces: Python, MATLAB
- Problem types: QP, NLP, ODE/DAE
- Free and open-source
- Efficient and flexible for optimization

**MPC Advantages:**
- Handles multivariable systems naturally
- Explicit constraint handling
- Predictive nature enables superior performance
- Can adapt to changing conditions

---

## Software Architecture

### Repository Structure

```
Reusable-Autonomous-VTOL-Vehicle/
├── README.md                           # This file
├── LICENSE                             # MIT License
├── Documents/
│   └── Arduino/
│       └── libraries/
│           └── major_project/          # Main firmware library
│               ├── MPU9250/            # IMU sensor interface
│               │   ├── MPU9250.cpp
│               │   └── MPU9250.h
│               ├── ahrs_ekf/           # Attitude estimation filter
│               │   ├── ahrs_ekf.cpp
│               │   └── ahrs_ekf.h
│               ├── pos_ekf/            # Position estimation filter
│               │   ├── pos_ekf.cpp
│               │   └── pos_ekf.h
│               ├── mpu_pose_ekf/       # Combined filter wrapper
│               │   ├── mpu_pose_ekf.cpp
│               │   └── mpu_pose_ekf.h
│               ├── gps/                # GPS module interface
│               │   ├── gps.cpp
│               │   └── gps.h
│               ├── bmp/                # Barometer interface
│               │   ├── bmp.cpp
│               │   └── bmp.h
│               ├── nrf/                # NRF24L01 telemetry
│               │   ├── nrf.cpp
│               │   └── nrf.h
│               ├── python_ahrs/        # Python visualization tools
│               │   ├── ekf_ahrs.py      # Attitude visualization
│               │   ├── position.py      # Trajectory visualization
│               │   ├── run_ekf_ahrs.bat
│               │   ├── run_position.bat
│               │   └── config/
│               │       └── config.prc
│               ├── teensy_examples/    # Reference implementation
│               │   └── ahrs_quat/
│               │       └── ahrs_quat.ino
│               └── readme.txt          # Legacy documentation
```

### Data Flow & Processing Pipeline

#### 1. **IMU Sampling (250 Hz)**
```
MPU9250 Hardware → Raw accel/gyro/mag data
                 ↓
         Bias compensation & calibration
                 ↓
        AHRS_EKF (attitude estimation)
                 ↓
        Quaternion output: q = [w, x, y, z]
```

#### 2. **GPS & Barometer (10 Hz)**
```
GPS Module → LLA coordinates (Lat, Lon, Alt)
Barometer → Pressure → Altitude via barometric formula
                 ↓
        Convert LLA to local NED coordinates
        (relative to startup reference point)
                 ↓
        Cache until next sample
```

#### 3. **Position Estimation (250 Hz internal, 10 Hz GPS update)**
```
Quaternion (from AHRS)
Body-frame acceleration (from MPU)
                 ↓
        Rotate acceleration to NED frame
        using rotation matrix from quaternion
                 ↓
        Integrate to get velocity & position
        (dead-reckoning between GPS updates)
                 ↓
        GPS update arrives (10 Hz)
                 ↓
        POS_EKF correction step
                 ↓
        Output: x = [pN, pE, pD, vN, vE, vD, aN, aE, aD]
        (position, velocity, acceleration in NED)
```

#### 4. **Control & Actuation**
```
Desired trajectory reference
Estimated state (from EKF)
                 ↓
        Control law (PID/LQR/MPC)
                 ↓
        Calculate required thrust & gimbal angle
                 ↓
        Send PWM commands to:
        - ESC (Electronic Speed Controller) for EDF thrust
        - Servo for thrust vectoring nozzle/vanes
```

#### 5. **Telemetry (via NRF24L01 - optional)**
```
State vector (quaternion + position/velocity)
                 ↓
        Pack 8 float values
                 ↓
        Transmit via NRF24L01 (2 Mbps)
                 ↓
        Ground station receives & visualizes
        (Python visualization scripts)
```

---

## Hardware Setup & Wiring

### Power Distribution
```
Battery (22.2V, 6S LiPo)
    ↓
LM2596 buck converter → 5V output
    ↓
AMS1117 regulator → 3.3V output
    ↓
Power Rails:
  - 5V: Actuators (servos, ESC)
  - 3.3V: Teensy, MPU9250, GPS, BMP280, NRF24L01
```

### Serial Communication
- **Serial1 (UART)**: GPS module (configurable baud, typically 38400)
- **Serial2 (optional)**: Additional telemetry or debug
- **Serial3+ (optional)**: Lidar or other sensors

### I2C Bus
- **SCL/SDA pins**: MPU9250, BMP280 (address: 0x68, 0x77 default)
- Pull-up resistors: 4.7 kΩ recommended
- Speed: 400 kHz standard I2C

### SPI Bus
- **MOSI/MISO/SCK**: NRF24L01, optional BMP280 SPI mode
- **CE (pin 9)**: NRF24L01 chip enable
- **CSN (pin 10)**: NRF24L01 chip select

### Servo PWM Output
- **Pin 20, 21, etc.**: Gimbal servo for thrust vectoring
- Typical range: 1000-2000 μs PWM pulse width

### ESC PWM Output
- **Pin 23 (or configurable)**: Electronic Speed Controller
- Range: 1000-2000 μs (1000 = off, 2000 = max thrust)

---

## Sensor Calibration & Compensation

### Accelerometer & Gyroscope Biases
At startup, perform bias estimation:
```
average_accel = mean(100 samples at rest)
bias_accel = average_accel - [0, 0, 9.81]  // subtract gravity
average_gyro = mean(100 samples at rest)
bias_gyro = average_gyro  // should be ~0
```

All subsequent measurements have biases subtracted.

### Magnetometer Calibration
Hard iron (offset) and soft iron (scale) correction:
```
m_corrected = C_m * (m_raw - b_m)

where:
  C_m = 3x3 scale matrix (calibrated via sphere-fitting)
  b_m = 3x1 hard iron offset (from min/max measurements)
```

### Barometric Altitude
Altitude from pressure via barometric formula:
$$h = \frac{T_0}{L} \left[ \left(\frac{P_0}{P}\right)^{R L / g M} - 1 \right]$$

Configuration: Sea-level pressure set to 1019.66 hPa (adjust for local conditions)

### GPS Initialization
Average first 50 GPS fixes to establish NED reference frame origin. All subsequent positions are relative to this reference point.

---

## Data Flow (Detailed)

### IMU Sampling (MPU9250)
- The system samples an MPU9250 IMU at a fixed rate (default: 250 Hz, $T_s = 0.004$ s).
- Accelerometer, gyroscope, and magnetometer data are acquired at each sample.

### Bias & Sensor Compensation
- At startup, bias calibration is performed by averaging initial samples.
- Accelerometer and gyroscope offsets are estimated and subtracted.
- Magnetometer measurements are corrected using ellipse (hard/soft iron) compensation.

### AHRS EKF
- An Attitude and Heading Reference System based on an Extended Kalman Filter (EKF) is used.
- Accelerometer, gyroscope, and magnetometer data are fused to estimate orientation.
- The filter outputs a normalized quaternion `q = [w, x, y, z]`.

### GPS & Barometer (BMP)
- GPS data is received using UBX NAV-PVT messages at 10 Hz.
- GPS provides position (latitude, longitude) and time information.
- A BMP280 barometer provides altitude measurements.
- Barometric altitude is sampled on the GPS update cycle for consistency.
- Vertical velocity is computed from successive barometric altitude differences.
- **Enhanced precision landing**: Lidar provides altitude when < 12m above ground.

### Position EKF (POS_EKF)
- Inputs include orientation `q` and body-frame accelerations `[ax, ay, az]`.
- EKF corrections are applied when the GPS `iTOW` value changes.
- GPS latitude, longitude, and altitude (LLA) are converted to local NED coordinates.
- The NED frame is defined relative to a startup reference point (first valid GPS fix).
- **NED Frame**: North (x), East (y), Down (z) relative to startup location.

### Telemetry
- Telemetry can be transmitted via an NRF24L01 radio link.
- The link is limited to 8 floating-point values per transmission.
- Typical telemetry includes quaternion (4 values) and NED position/velocity (3+ values).
- Ground station receives data at 10+ Hz for real-time visualization.

### Visualization
- Python scripts are used for visualization and analysis.
- Orientation is displayed using an attitude cube (3D rotation visualization).
- Motion is visualized as a 3D flight trajectory or flight path map.
- Real-time plotting of position, velocity, altitude, and orientation.

---

## Hardware Components Summary

### Microcontroller
- **Teensy 4.1** or Arduino-compatible with `Serial1`, I2C, SPI
- Recommended for this project: Teensy 4.1 (600 MHz, 2 MB flash, 8 hardware serials)

### Sensors
- **MPU9250** IMU: I2C default address `0x68`
- **u-blox NEO-M8N** GPS: UART on `Serial1`
- **BMP280** Barometer: I2C address `0x77`
- **TFmini Lidar**: UART on `Serial2` (optional, for precision landing)
- **NRF24L01** 2.4 GHz Radio: SPI with CE=9, CSN=10 (optional)

### Power Supplies
- Primary: 6S LiPo battery (22.2V nominal)
- LM2596 buck converter: 22V → 5V
- AMS1117 regulator: 5V → 3.3V
- Total current budget: ~10-15 A during thrust (peak)

---

## Software Requirements

### Embedded (Arduino/Teensy)
- **Teensy 4.1** board support installed
- Libraries:
  - `Adafruit_BMP280` – barometer interface
  - `RF24` / `nRF24L01` – wireless telemetry
  - `Wire` / `SPI` – I2C/SPI core protocols
  - `Eigen` – matrix math (shim header included in project)

### Python (Optional Visualization)
- **Python 3.7+**
- Core packages:
  - `ahrs` – AHRS filter library (for reference/comparison)
  - `panda3d` – 3D visualization engine
  - `pyserial` – serial communication with flight computer
  - `matplotlib` – 2D plotting of trajectories and data
  - `numpy` – numerical computing

---

## Setup and Usage

### Arduino/Teensy Library Installation

1. **Install Teensy Board Support:**
   - Download Teensyduino from https://www.pjrc.com/teensy/teensyduino.html
   - Follow installation instructions for your IDE (Arduino or VS Code with extensions)

2. **Place the firmware library:**
   - Copy `major_project/` folder to your Arduino libraries directory:
     - Windows: `Documents\Arduino\libraries\`
     - macOS: `~/Documents/Arduino/libraries/`
     - Linux: `~/Arduino/libraries/`

3. **Install dependencies:**
   - Open Arduino IDE → Sketch → Include Library → Manage Libraries
   - Search and install:
     - `Adafruit BMP280 Library`
     - `RF24` (by TMRh20)
     - Ensure `Wire` and `SPI` are available (usually included)

4. **Set up header paths:**
   - Ensure your project sketch can find `ahrs_ekf.h`, `pos_ekf.h`, etc.
   - Or use the provided `teensy_examples/ahrs_quat/ahrs_quat.ino` as a template

### Sensor Initialization

**In your setup() function, call pose_setup() once:**

```cpp
void setup() {
  Serial.begin(115200);      // Debug serial
  pose_setup();              // Initialize all sensors
  // Internally:
  // - Calls imu_setup() to configure MPU9250 ranges/bandwidth
  // - Reads IMU to estimate initial quaternion from accel + mag
  // - Runs gps_setup() and bmp_setup()
  // - Averages initial GPS & barometer readings for NED reference
}
```

### Main Loop Update

**Call pose_update() repeatedly in your loop() function:**

```cpp
void loop() {
  pose_update();
  
  // After pose_update(), the following variables are available:
  // q[0..3]        : attitude quaternion [w, x, y, z]
  // x[0..8]        : [aN, aE, aD, vN, vE, vD, pN, pE, pD] in NED
  // pos_ecef[0..2] : position in ECEF (optional)
  
  // Example: send telemetry
  if (new_gps_data) {
    sendTelemetry(q, x);
  }
  
  // Example: control law (LQR/PID/MPC)
  controlAndActuate(q, x);
}
```

### Output Variables

**Attitude:**
- `q[0]` = w (scalar part of quaternion)
- `q[1]` = x, `q[2]` = y, `q[3]` = z (vector part)
- Quaternion is normalized: $\|q\| = 1$

**Position & Velocity (NED frame in meters, m/s):**
- `x[6]` = pN, `x[7]` = pE, `x[8]` = pD (North, East, Down position)
- `x[3]` = vN, `x[4]` = vE, `x[5]` = vD (North, East, Down velocity)
- `x[0]` = aN, `x[1]` = aE, `x[2]` = aD (North, East, Down acceleration)

### Python Visualization

**Real-time attitude and trajectory plotting:**

1. **Edit configuration** (optional):
   ```
   python_ahrs/config/config.prc  # Set graphics parameters
   ```

2. **Edit serial port** in visualization scripts:
   ```python
   # In ekf_ahrs.py and position.py, set:
   PORT = "COM10"              # Windows
   BAUDRATE = 2000000          # Match your serial settings
   ```

3. **Run visualization:**
   ```bash
   cd Documents/Arduino/libraries/major_project/python_ahrs/
   python ekf_ahrs.py          # Shows 3D attitude cube
   # or
   python position.py          # Shows flight trajectory on map
   ```

   Or use the batch files:
   ```
   run_ekf_ahrs.bat            # Windows
   run_position.bat            # Windows
   ```

**Expected serial format** (comma-separated, one line per update):
```
timestamp, qw, qx, qy, qz, pN, pE, pD
1000.45, 0.98, 0.05, -0.02, 0.01, 10.5, -3.2, 5.1
```

---

## Configuration Parameters

### IMU Configuration (`MPU9250.h`)
```cpp
// Accelerometer range (g)
#define MPU_ACCEL_RANGE  16          // ±16g (or 2, 4, 8)

// Gyroscope range (°/s)
#define MPU_GYRO_RANGE   2000        // ±2000°/s (or 250, 500, 1000)

// Digital Low Pass Filter bandwidth (Hz)
#define MPU_DLP_BW       42          // 5 Hz, 10, 21, 44, 94, 111, 184, 260
```

### GPS Configuration (`gps.h`)
```cpp
// GPS sample rate (Hz)
#define GPS_RATE         10          // 10 Hz (via UBX config)

// Sea-level pressure (hPa) - adjust for your location
#define SEA_LEVEL_PRESSURE  1019.66
```

### Barometric Altitude (`bmp.h`)
```cpp
// Sea-level pressure for altitude calculation
#define SEA_LEVEL_PRESSURE  1019.66  // Standard: 1013.25 hPa
                                     // Adjust for local atmospheric pressure
```

### Filter Tuning
- **AHRS_EKF**: Process and measurement noise covariances in `ahrs_ekf.cpp`
- **POS_EKF**: State and measurement uncertainties in `pos_ekf.cpp`
- **Telemetry rate**: Set in main sketch (typically 10-50 Hz)

---

## Known Issues and Caveats

### Sensor-Specific Issues

1. **Magnetometer Interference:**
   - Sensitive to magnetic fields from nearby electronics and wiring
   - Solution: Position magnetometer away from battery/EDF wiring; enclose in mu-metal shielding if needed

2. **Barometric Altitude Drift:**
   - Pressure sensors drift over time and with temperature
   - Solution: Re-calibrate sea-level pressure regularly; use Lidar for precision near ground

3. **GPS Lock Acquisition:**
   - May take 30-60 seconds on cold start
   - Solution: Implement watchdog timer; handle GPS unavailability gracefully in filters

4. **NRF24L01 Range Limitations:**
   - Effective range ~100 m in open space
   - May be reduced by obstacles and RF interference
   - Solution: Test link budget before flight; consider relay stations for long-range flights

### Software Considerations

1. **Time Synchronization:**
   - IMU sampled at 250 Hz, GPS at 10 Hz, asynchronous updates
   - Ensure accurate timestamps for data fusion
   - GPS iTOW value (integer time of week) used for synchronization

2. **Quaternion Singularities:**
   - Gimbal lock not an issue with quaternions, but ensure normalization
   - Verify $\|q\| \approx 1$ during integration

3. **Numerical Stability:**
   - Matrix inversions in EKF can be numerically sensitive
   - Use Cholesky or QR decompositions for robustness (see Eigen documentation)

4. **Caching of Sensor Data:**
   - MPU9250 data cached between filter updates
   - GPS data averaged over multiple sentences before position EKF update
   - Ensures consistent timing and reduces computational load

### System Limitations

- **Linear approximation for control design**: While the vehicle dynamics are nonlinear, the control algorithms assume linearized models. This may limit performance at high angles of attack.
- **Kalman filter optimality**: EKF is optimal only for linear systems; performance degrades for highly nonlinear dynamics.
- **Computational constraints**: Teensy runs at 600 MHz; ensure sufficient margin for real-time processing.
- **Power budget**: High current draw during thrust may cause voltage sag; ensure adequate power distribution.

---

## Future Enhancements

### Sensor Upgrades
- **Multi-antenna GPS**: For accurate heading estimation
- **IMU redundancy**: Multiple IMUs for fault tolerance
- **Sun sensors**: For attitude reference in daylight
- **Star trackers**: For precision attitude in space environments

### Advanced Control
- **Nonlinear MPC**: Better handling of system nonlinearities
- **Adaptive control**: Automatic tuning for changing flight conditions
- **Fault-tolerant control**: Graceful degradation if sensors fail
- **Trajectory optimization**: Real-time optimal path planning

### System Improvements
- **Hardware**: Onboard computing for MPC (NVIDIA Jetson, etc.)
- **Cloud telemetry**: Upload flight data for post-flight analysis
- **Distributed control**: Multiple vehicles with cooperative control
- **Machine learning**: Neural network-based state estimation or control

---

## Contributing & Acknowledgments

### Contributing
- Open issues and pull requests are welcome
- Please include hardware setup, logs, and steps to reproduce for bug reports
- For new sensors or control algorithms, maintain consistent units and document thoroughly

### Acknowledgments
- **MPU9250 Driver**: Bolder Flight Systems (GPL-3.0 licensed)
- **Python AHRS Library**: https://ahrs.readthedocs.io/
- **CasADi**: https://web.casadi.org/ (open-source optimal control)
- **IPOPT**: https://coin-or.github.io/Ipopt/ (nonlinear optimization)

### References

[1] Sørensen et al. (2017). Flexible UAV Sensor Deployment Using Teensy Microcontrollers.

[2] Pascucci, Bennani, Bemporad (2015). MPC for Precision Landing with Thrust Vectoring.

---

## License

**MIT License** - See [LICENSE](LICENSE) file for details.

This project is provided as-is for educational and research purposes. Use at your own risk.

---

## Final Notes

This README serves as both a user guide and technical reference. For code-level documentation, see header comments in `.cpp` and `.h` files.

**Happy flying!** 🚀

