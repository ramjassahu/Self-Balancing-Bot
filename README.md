# Self-Balancing Robot

![IIT Guwahati](https://img.shields.io/badge/IIT-Guwahati-blue)

A Control and Systems Design Lab Project (EE396) developed at the Indian Institute of Technology Guwahati.

**Supervised by:** Dr. Chayan Bhawal  
**Developed by:** Ramjas Sahu (220108048), Raghav Mour (220108047)

---

► [**Read the Full Project Report (PDF)**](https://drive.google.com/file/d/1nAfDRQbDrQ6y6m6SppmXOfBGuh5Kue7e/view?usp=sharing)  
► [**Watch the Live Demo**](https://drive.google.com/file/d/1RLFNf0l0xZw-rqtDqjI63CZG3am7M6Hq/view?usp=sharing)

---

## 🤖 Abstract

This project details the design and construction of a two-wheeled self-balancing robot. The robot utilizes an **Arduino Uno**, an **MPU6050** gyroscope and accelerometer, and an **L293D** motor driver to maintain its balance. The core of the system is a **Proportional-Integral-Derivative (PID) control algorithm**, which processes sensor data to dynamically adjust motor speeds and keep the robot upright. This project serves as a practical demonstration of feedback control systems in robotics.



---

## 🛠️ Components Required

| Component         | Description                                                                     |
| ----------------- | ------------------------------------------------------------------------------- |
| **Arduino Uno** | The microcontroller brain of the robot, running the PID control algorithm.      |
| **MPU6050** | A 6-DOF Accelerometer & Gyroscope module to measure the robot's tilt angle.   |
| **L293D Motor Driver** | An IC that receives control signals from the Arduino to drive the two DC motors. |
| **Robot Chassis** | A custom 3D printed chassis with two N20 Encoder Motors.                        |
| **Power Supply** | A battery pack to power the Arduino, sensors, and motors.                       |
| **Jumper Wires** | For making connections between components.                                      |

---

## 🔩 Hardware Design

### 3D Printed Chassis
A lightweight and compact body was designed and 3D printed to house all the electronic components securely. The dimensions (**10 cm × 6 cm × 4 cm**) were chosen to keep the center of mass low and close to the wheel axle, which is critical for stability.

### Circuit and Connections
The MPU6050 communicates with the Arduino via the I2C protocol (SDA to A4, SCL to A5). The Arduino sends PWM signals to the L293D motor driver, which in turn controls the speed and direction of the N20 motors.

**Complete Fritzing Diagram:**
*Add your Fritzing diagram image here.*


---

## ⚙️ Control System: PID Controller

A PID (Proportional-Integral-Derivative) controller is a feedback control loop mechanism that calculates an "error" value as the difference between a desired setpoint and the measured process variable. It applies a correction based on proportional, integral, and derivative terms.

The PID control equation is:
$$ \text{output}(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt} $$

-   **Proportional (P):** Corrects the present error. A higher $K_p$ results in a faster response but can lead to instability.
-   **Integral (I):** Corrects the accumulation of past errors, eliminating steady-state error.
-   **Derivative (D):** Predicts future errors based on the current rate of change, helping to dampen oscillations.

### System Response
The PID parameters ($K_p, K_i, K_d$) were manually tuned to achieve a stable response. The graph below shows the robot's angle stabilizing around the setpoint (upright position) over time.

*Add your system response graph image here.*


---

## 💻 Software

The complete Arduino source code can be found in the `src/` directory.

### Libraries Used
* `PID_v1.h`
* `LMotorController.h`
* `I2Cdev.h`
* `MPU6050_6Axis_MotionApps20.h`
* `Wire.h`

---

## 🚀 Procedure

1.  **Assemble the Chassis:** Mount the motors, Arduino, MPU6050, and L293D driver onto the 3D printed body.
2.  **Wire Components:** Connect all components according to the circuit diagram.
3.  **Upload Code:** Open the `.ino` file in the Arduino IDE, install the required libraries, and upload the sketch to the Arduino Uno.
4.  **Power On:** Connect the battery pack to power the system.
5.  **Tune PID:** If necessary, adjust the `Kp`, `Ki`, and `Kd` values in the code to optimize the balance for your specific robot's weight and geometry.

---

## ✅ Conclusion

The project successfully resulted in a functional self-balancing robot. Through careful assembly and PID tuning, the robot was able to maintain its balance on two wheels, demonstrating the effectiveness of PID control in real-world applications. Future improvements could include using more powerful motors, a more accurate IMU, or implementing remote control capabilities.

---

## 📚 References

* [Demo Video](https://drive.google.com/file/d/1RLFNf0l0xZw-rqtDqjI63CZG3am7M6Hq/view?usp=sharing)
* [NIT Rourkela E-Thesis on Self Balancing Robot](https://drive.google.com/drive/folders/1RNJAxQK0Tnb0RRvQr07kv5jTSc4IZ7zL)
* [Maker.pro Arduino Self Balancing Robot](https://drive.google.com/drive/folders/1RNJAxQK0Tnb0RRvQr07kv5jTSc4IZ7zL)
* [Instructables: Self-Balancing Robot](https://drive.google.com/drive/folders/1RNJAxQK0Tnb0RRvQr07kv5jTSc4IZ7zL)
