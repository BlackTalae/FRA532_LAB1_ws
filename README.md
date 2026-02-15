# **FRA532 LAB1: Kalman Filter/ SLAM Report**

## **Author**
- 65340500058 Anuwit Intet

## **Introduction**
This project aims to study and develop robot localization and mapping procedures, divided into four main parts:
- **Part 0:** Calculating Wheel Odometry to determine the robot's position using the robot's wheel positions.
- **Part 1:** Performing Sensor Fusion between Wheel Odometry and IMU using an Extended Kalman Filter (EKF) to reduce errors caused by wheel rotation and slippage.
- **Part 2:** Position improvement using ICP Scan Matching, using EKF values ​​as the initial guess to generate LiDAR-based odometry.
- **Part 3:** Performing Full SLAM with slam_toolbox to compare the performance of loop closure and complete map generation.

## **Usage**

**How to run this project.**

## **Dataset Description**
The dataset is provided as a ROS bag and contains sensor measurements recorded during robot motion.

Topics included:

- **/scan**: 2D LiDAR laser scans at **5 Hz**
- **/imu**: Gyroscope and accelerometer data at **20 Hz**
- **/joint_states**: Wheel motor position and velocity at **20 Hz**

The dataset is divided into three sequences, each representing a different environmental condition:

**Sequence 00 – Empty Hallway:** A static indoor hallway environment with minimal obstacles and no dynamic objects. This sequence is intended to evaluate baseline odometry and sensor fusion performance.

**Sequence 01 – Non-Empty Hallway with Sharp Turns:** An indoor hallway environment containing obstacles and clutter, with sections of sharp turning motion. This sequence is designed to challenge odometry and scan matching performance under rapid heading changes.

**Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion:** An indoor hallway environment with obstacles, similar to Sequence 2, but recorded with smoother and non-aggressive robot motion. This sequence is intended to evaluate performance under more stable motion conditions.

## **Part 0: Calculating Wheel Odometry**

### **Objective**
To establish a baseline reference and study the impact of cumulative drift caused solely by the mechanical system.

### **Theory**

There are two main ways to calculate the odometry of a differential drive robot:

**Type 1: Position-based Odometry**

This method calculates the difference in wheel angular position ($\Delta \phi$) over time, which helps reduce the accumulation of time jitter error in the ROS 2 system.

Calculate wheel distance:

$$\Delta d_{left} = R \cdot (\phi_{L, t} - \phi_{L, t-1})$$

$$\Delta d_{right} = R \cdot (\phi_{R, t} - \phi_{R, t-1})$$

$$\Delta d = \frac{(\Delta d_{right} + \Delta d_{left})}{2}$$

$$\Delta \theta = \frac{(\Delta d_{right} - \Delta d_{left})}{L}$$

Update position:

$$x_{t+1} = x_{t} + \Delta d \cos(\theta_{old} + \frac{\Delta \theta}{2})$$

$$y_{t+1} = y_{t} + \Delta d \sin(\theta_{old} + \frac{\Delta \theta}{2})$$

$$\theta_{t+1} = \theta_{t} + \Delta \theta$$

**Type 2: Velocity-based Odometry**

This method uses the angular velocity ($\omega_{wheel}$) to calculate the velocity of the robot directly, which is suitable for use in the EKF prediction step as it provides smooth velocity calculations for the robot.

Calculate velocity:

$$v = \frac{R}{2} (\omega_R + \omega_L)$$

$$\omega = \frac{R}{L} (\omega_R - \omega_L)$$

Update position with Integration:

$$x_{t+1} = x_t + v \cos(\theta_t) \Delta t$$

$$y_{t+1} = y_t + v \sin(\theta_t) \Delta t$$

$$\theta_{t+1} = \theta_t + \omega \Delta t$$

### **Methodology**

- **Data Source**: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /joint_states.

- **Implementation**: Calculate odometry using both Position-based (from wheel position) and Velocity-based (from wheel velocity) methods, using wheel radius (R) = 0.033 meters and wheel base (L) = 0.16 meters to compare the differences between the two methods in all 3 sequences.

- **Path Visualization**: Plot the path of movement that is calculated from the sum of the coordinates $(x, y)$ calculated from /joint_states to compare with the scan points

- **Usage**: You can run this file to see the test results
    - wheel_odom_node.py

- **Experiment**: Perform Position-based and Velocity-based wheel odometry in all 3 sequences and analyze accuracy, drift, and robustness.

### **Result & Discussion**

**Green Line:** Position-based Odometry

**Yellow Line:** Velocity-based Odometry

``Sequence 00 Empty Hallway``

![Path - Sequence 00](pic/part0/seq00_path_all.png)

**Analysis**

- **Heading Drift**: Both modes exhibit clear heading drift when compared to the grid lines. The final coordinates fail to converge back to the starting point (closure error).

- **Path Smoothness**: The Position Mode (Green) path shows higher jitter compared to Velocity Mode (Yellow), indicating lower robustness to environmental conditions.

- **Turning Accuracy**: The green line maintains sharper rectangular corners in some instances, while the yellow line exhibits over-smoothed curves at corners.

---

``Sequence 01 Non-Empty Hallway with Sharp Turns``

![Path - Sequence 01](pic/part0/seq01_path_all.png)

**Analysis**

- **Heading Drift**: Both modes show severe heading drift, causing significant distortion of the rectangular shape when compared to the grid lines.

- **Path Smoothness**: Position Mode (Green) exhibits high jitter and path oscillations throughout the trajectory, reflecting lower robustness to environmental noise compared to Velocity Mode (Yellow), which maintains better continuity and smoothness.

- **Turning Accuracy**: Position Mode (Green) better preserves the rectangular shape at corners despite high noise levels, while Velocity Mode (Yellow) shows over-smoothed curves at all turns.

---

``Sequence 02 Non-Empty Hallway with Non-Aggressive Motion``

![Path - Sequence 02](pic/part0/seq02_path_all.png)

**Analysis**

- **Heading Drift**: Both modes show clear heading drift when compared to grid lines. The final coordinates fail to converge to the starting point (closure error). Additionally, there is severe accumulated "orientation loss" in certain sections, causing the rectangular shape to deviate significantly from the main axis.

- **Path Smoothness**: Position Mode (Green) consistently shows higher jitter compared to Velocity Mode (Yellow), indicating lower robustness to environmental conditions and noise. During moments of swaying or vibration, the green path responds with more severe oscillations.

- **Turning Accuracy**: The green line maintains sharper rectangular corners in some instances despite high noise levels, while the yellow line exhibits over-smoothed curves at corners.

---

``Summary``

**Heading Drift & Closure Error**: Both modes show clear accumulated error when compared to grid lines, with the final coordinates deviating significantly from the starting point, especially in sequences 01 and 02.

**Path Smoothness & Robustness**: Velocity Mode (Yellow) provides a significantly smoother path, while Position Mode (Green) exhibits high jitter, indicating lower robustness to noise.

**Turning Accuracy & Tuning**: Although the green line better preserves sharp rectangular corners in some instances, the over-smoothed curves at corners in the yellow line are "easier to tune and correct" by improving yaw accuracy from other sensors, which will be addressed in subsequent development.

**Conclusion**: Velocity Mode (yellow) is the most suitable choice for integration with EKF because EKF will compensate for its weaknesses in heading drift and over-smoothed turns, while benefiting from the smooth path as a good initial estimate.

## **Part 1: Sensor Fusion between Wheel Odometry and IMU using an Extended Kalman Filter (EKF)**

### **Objective**

Develop a state estimator using the Extended Kalman Filter (EKF) to reduce rotational errors that are often highly pronounced in wheel-only systems.

### **Theory**

In this section, we use an EKF to perform sensor fusion between wheel odometry (as the control input) and IMU yaw (as the measurement) to improve positioning accuracy.

**1. Prediction Step (Time Update)**

This step involves predicting the new state of the robot based on the kinematic model:

State Prediction:

$$\hat{x}_{t} = f(\hat{x}_{t-1}, u_t) = \begin{bmatrix} x_{t-1} + v \cos(\psi_{t-1}) \Delta t \\ y_{t-1} + v \sin(\psi_{t-1}) \Delta t \\ \psi_{t-1} + \omega \Delta t \end{bmatrix}$$

Covariance Prediction:

$$P_{t|t-1} = G_t P_{t-1} G_t^T + Q$$

**2. Update Step (Measurement Update)**

This step involves using the measured values from the sensor (IMU) to improve the predicted values:

Innovation (Measurement Residual):

$$y_t = z_t - H \hat{x}_{t}$$

Kalman Gain:

$$K_t = P_{t|t-1} H^T (H P_{t|t-1} H^T + R)^{-1}$$

State & Covariance Update:$$\hat{x}_t = \hat{x}_{t|t-1} + K_t y_t$$

$$P_t = (I - K_t H) P_{t|t-1}$$

**Definition of Variables**

$\hat{x}$ State Vector

$P$ Covariance Matrix - Shows the uncertainty of the robot's state.

$u_t$ Control Input

$G_t$ Jacobian Matrix - Linear estimation matrix of the motion model.

$Q$ Process Noise - Confidence in the model

$z_t$ Measurement - The actual measured value, in this case, is the yaw angle from the IMU.

$H$ Observation Model - Matrix that specifies which sensor measures which state (in this case, only $Yaw$)

$R$ Sensor Noise - Confidence in the sensor

$K_t$ Kalman Gain - Weighting factor between "predicted value" and "actual measured value"

**To determine the position of a two-wheeled robot, each matrix can be defined as follows:**

``State Vector``
$$\hat{x}_{t} = f(\hat{x}_{t-1}, u_t) = \begin{bmatrix} x_{t-1} + v \cos(\psi_{t-1}) \Delta t \\ y_{t-1} + v \sin(\psi_{t-1}) \Delta t \\ \psi_{t-1} + \omega \Delta t \end{bmatrix}$$

``Jacobian Matrix``

$$G_t = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} \frac{\partial f_x}{\partial x} & \frac{\partial f_x}{\partial y} & \frac{\partial f_x}{\partial \psi} \\ \frac{\partial f_y}{\partial x} & \frac{\partial f_y}{\partial y} & \frac{\partial f_y}{\partial \psi} \\ \frac{\partial f_\psi}{\partial x} & \frac{\partial f_\psi}{\partial y} & \frac{\partial f_\psi}{\partial \psi} \end{bmatrix} = \begin{bmatrix} 1 & 0 & -v \sin(\psi) \Delta t \\ 0 & 1 & v \cos(\psi) \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

``Process Noise Matrix``
$$Q = \begin{bmatrix} \sigma_x^2 & 0 & 0 \\ 0 & \sigma_y^2 & 0 \\ 0 & 0 & \sigma_\psi^2 \end{bmatrix}$$

- Model reliability coefficient
    - If the value is low, the system trusts the model more.
    - If the value is high, the system trusts the IMU more.

``Sensor Noise Matrix``
$$R = [\sigma_{imu\_yaw}^2]$$

- $\sigma_{imu\_yaw}^2$: The variance of the IMU signal. 
    - **Small value:** The system will trust the IMU more (the path will be straighter, but may wobble due to sensor noise). 
    - **Large value:** The system will trust it less (the path will be smooth, but will accumulate drift easily).

``Initial State Covariance Matrix``
$$P_0 = \begin{bmatrix} p_{xx} & 0 & 0 \\ 0 & p_{yy} & 0 \\ 0 & 0 & p_{\psi\psi} \end{bmatrix}$$

``Observation Model Matrix``
$$H = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}$$

In this case, the observation model is only $Yaw$

``Measurement``

In this case, the measurement is the yaw angle from the IMU.

### **Setup**

- Data Source: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /joint_states for wheel odometry and /imu for EKF measurement.

- Implementation: 
    - Calculate wheel odometry using Velocity-based (from wheel velocity) methods as a initial guess, using wheel radius (R) = 0.033 meters and wheel base (L) = 0.16 meters. 
    - Perform EKF to improve the accuracy of the wheel odometry by using /joint_state and angular velocity from /imu as a prediction step and orientation from /imu as a update step.

- Path Visualization: Plot the path of movement from EKF in each timestep on rviz2.

- Usage: You can run this file to see the test results
    - EKF_odom_node.py

- Experiment: Perform EKF in variance of $Q$ and $R$ in all 3 sequences and analyze accuracy, drift, and robustness.
    - Experiment 1: $Q$ = 0.000001, 0.001, 1.0, 1000.0 and $R$ = 0.1
    - Experiment 2: $R$ = 0.0001, 0.1, 1000.0, 100000.0 and $Q$ = 0.001
    - Choose the best $Q$ and $R$ and compare the results to Velocity-based odometry.

### **Result & Discussion**

``Experiment 1: Adjust process noise``

R = 0.1

**Yellow Line**: Q = 0.000001

**Green Line**: Q = 0.001

**Cyan Line**: Q = 1.0

**Red Line**: Q = 1000.0

**Sequence 00**
![Adjust Process Noise](pic/part1/Seq00_adjustQ_1.png)
![Adjust Process Noise](pic/part1/Seq00_adjustQ_2.png)

**Sequence 01**
![Adjust Process Noise](pic/part1/Seq01_adjustQ_1.png)
![Adjust Process Noise](pic/part1/Seq01_adjustQ_2.png)

**Sequence 02**
![Adjust Process Noise](pic/part1/Seq02_adjustQ_1.png)
![Adjust Process Noise](pic/part1/Seq02_adjustQ_2.png)

**Analysis:**

**Geometric Fidelity**: The green line demonstrates the highest performance in maintaining the "rectangular shape," with sharper and more perpendicular corners compared to other lines. The yellow line exhibits more "turning deviation" than other colors.

**Jitter vs. Stability**: The yellow line shows significant path oscillations or "shaking" throughout the trajectory, caused by low process noise ($Q$) settings, making the system more sensitive to sensor noise.

**Heading & Drift**: In straight sections, the green line demonstrates the most responsive heading change capability, similar to cyan and red lines, making the overall path most consistent with the robot's intended rectangular motion pattern.

**Conclusion**: When measured by rectangular shape and geometric accuracy, the green line performs best due to appropriately balanced process noise ($Q$). Setting process noise too low makes the system overly sensitive to sensor noise.

---

``Experiment 2: Adjust measurement noise``

Q = 0.001

**Yellow Line**: R = 0.0001

**Green Line**: R = 0.1

**Cyan Line**: R = 1000.0

**Red Line**: R = 100000.0

**Sequence 00**
![Adjust Measurement Noise](pic/part1/Seq00_adjustR_1.png)
![Adjust Measurement Noise](pic/part1/Seq00_adjustR_2.png)

**Sequence 01**
![Adjust Measurement Noise](pic/part1/Seq01_adjustR_1.png)
![Adjust Measurement Noise](pic/part1/Seq01_adjustR_2.png)

**Sequence 02**
![Adjust Measurement Noise](pic/part1/Seq02_adjustR_1.png)
![Adjust Measurement Noise](pic/part1/Seq02_adjustR_2.png)

**Analysis:**

From analyzing all 3 sequences with emphasis on "rectangular squareness" (Geometric Squareness):

**Geometric Fidelity**: The green (lime) line performs best in maintaining "90-degree right angles" and sharp rectangular shapes, while cyan and red lines exhibit "turning deviation."

**Jitter vs. Stability**: Although the green line maintains the best rectangular shape, it comes at the cost of "roughness" or "shaking" (High Jitter) throughout the path.

**Heading & Drift**: In straight sections, both green and yellow lines maintain parallelism with grid lines quite similarly. However, the green line shows more responsive and accurate heading changes, resulting in a noticeably more rectangular overall shape across all sequences.

**Conclusion**: When measured by rectangular shape and geometric accuracy, green and yellow lines perform best (paths overlap). This is due to appropriately low measurement noise ($R$). Setting measurement noise too high prevents the system from trusting sensor readings, causing over-reliance on the model. Since the model alone cannot achieve perfect rectangular motion, lower $R$ values allow the system to trust the sensor more.

---

``Compare the results to Velocity-based odometry``

**Yellow Line**: EKF Odometry

**Green Line**: Velocity-Based Wheel Odometry

**Sequence 00**
![Compare to Velocity-based odometry](pic/part1/Seq00_compare_path.png)

**Sequence 01**
![Compare to Velocity-based odometry](pic/part1/Seq01_compare_path.png)

**Sequence 02**
![Compare to Velocity-based odometry](pic/part1/Seq02_compare_path.png)

**Analysis:**

From comparing Velocity-based Wheel Odometry (green line) and EKF Odometry (yellow line) across different scenarios:

**Heading Drift and Angular Accuracy**: The yellow line (EKF) demonstrates superior heading maintenance compared to the green line across multiple sequences. Particularly during cornering maneuvers requiring parallelism with grid lines, the yellow line shows less distortion at the beginning of each segment.

**Path Smoothness and Noise Robustness**: The green line (Velocity-based), while continuous, exhibits more "roughness" or path inconsistency (jitter) in certain sections compared to the yellow line.

**Turning Accuracy and Geometric Shape**: During 90-degree turns, the green line may preserve corner sharpness better at some points but often comes with overshooting or jerky coordinates. Meanwhile, the yellow line (EKF) maintains balance between turning and continuity, resulting in a more symmetrical and stable overall rectangular shape.

**Conclusion**: The yellow line (EKF Odometry) is superior because EKF can integrate additional data (e.g., from IMU) to correct heading, overcoming the limitations of Wheel Odometry which tends to drift easily when wheel slippage occurs.

