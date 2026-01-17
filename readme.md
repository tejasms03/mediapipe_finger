# Gesture-Controlled Mechatronic Finger  
**Vision-Based Human-in-the-Loop Robotic Finger with Model-Based Control**

---

## 📌 Project Overview

This project presents the design, modeling, simulation, and validation of a **vision-based, gesture-controlled single-degree-of-freedom (DOF) mechatronic finger**. The system translates **human finger motion captured by a camera** into real-time actuation of a robotic finger through **embedded motor control and feedback stabilization**.

The work integrates **computer vision**, **embedded systems**, **control theory**, and **mechanical design** into a unified mechatronic system. A reduced-order single-DOF finger was intentionally chosen to enable rigorous modeling and controller validation while serving as a scalable foundation for future multi-finger robotic hands.

---

## 🎯 Objectives

- Enable **intuitive human–robot interaction** via vision-based gesture control  
- Design a **physically grounded electromechanical model**  
- Implement **closed-loop motor control** with embedded hardware  
- Validate control performance via **simulation and real-time experiments**  
- Create a **model-based simulation** mirroring real hardware behavior  

---

## 🧩 System Architecture

**High-level pipeline:**

```
Camera → Gesture Detection → Reference Generation → 
Embedded Controller → Motor Driver → Tendon Mechanism → Robotic Finger
```

The system operates as a **human-in-the-loop closed-loop control system**, where human motion acts as a dynamic reference input.

---

## 🛠️ Work Done (Technical Breakdown)

### 1️⃣ Computer Vision & Gesture Processing
- Real-time hand landmark tracking using **MediaPipe**
- Finger joint angle estimation from landmark geometry
- Noise reduction via filtering and deadband logic
- Continuous reference signal generation from human motion

---

### 2️⃣ Reference Mapping & Signal Conditioning
- Mapping finger bend percentage to motor angle reference
- Saturation limits to ensure safe operation
- Smooth trajectory generation to avoid abrupt motion

---

### 3️⃣ Embedded Firmware & Real-Time Control
- Arduino firmware for:
  - Quadrature encoder feedback
  - Serial communication with host PC
  - PWM motor actuation
- Closed-loop **PD position control**
- Safety handling (timeouts, saturation, fault detection)

---

### 4️⃣ Mathematical Modeling & Control Design
- DC motor electromechanical modeling
- Tendon-driven finger dynamics modeled as a **SISO plant**
- PD controller design and tuning
- Stability and performance verification via simulation

---

### 5️⃣ Mechanical Design & Fabrication
- Finger, tendon routing, and motor mount designed in **Fusion 360**
- Tendon-driven flexion with elastic dorsal return
- Iterative mechanical refinement
- Fabrication using **3D printing**

---

## 🧪 Simulation & Model-Based Control Documentation

### 📄 Simulation Script
```
full_sim_with_model_based_control.py
```

This simulation provides a **real-time physics-inspired environment** for validating control strategies using **live human hand gestures**.

---

### 🧠 Simulation Features

#### Vision Integration
- Webcam input via OpenCV
- MediaPipe hand skeleton extraction
- Real-time finger bend estimation

#### Trajectory Generation
- Quintic polynomial trajectories
- Smooth position, velocity, and acceleration references
- Robust adaptation to changing gestures

#### Model-Based Control
- Computed torque feedforward:
  
  ```
  τ = J·φ̈_ref + b·φ̇_ref + k·φ_ref
  ```

- PD feedback control
- Actuator torque saturation
- Dynamic plant integration

#### Mechanical Visualization
- Three-link finger representation
- Tendon-driven actuation
- Elastic dorsal springs
- Tendon break and reattachment simulation
- Motor pulley visualization

---

### ▶️ How to Run the Simulation

#### 1️⃣ Install Dependencies
```bash
pip install numpy pygame opencv-python mediapipe
```

Linux users may also need:
```bash
sudo apt install libsdl2-dev
```

---

#### 2️⃣ Run the Simulation
```bash
python full_sim_with_model_based_control.py
```

---

#### 3️⃣ Usage Instructions
- Move your **index finger** in front of the camera
- The simulated finger mirrors your motion in real time
- UI buttons:
  - **Break Tendon** — simulate mechanical failure
  - **Reattach Tendon** — restore actuation
- Monitor:
  - Joint angles
  - Motor torque
  - Bend percentage
  - Control response

---

### 🧪 Simulation Assumptions
- Lumped inertia and damping
- Linear spring behavior
- Quasi-static tendon force distribution
- Single-DOF motor approximation
- Neglects friction nonlinearities

All assumptions are displayed in the UI for transparency.

---

## 🧠 Technical Domains Covered

- Computer Vision  
- Control Systems  
- Embedded Systems  
- Mechatronics  
- Robotics  
- Human–Robot Interaction  
- Model-Based Control  
- Simulation & Visualization  

---

## 🚀 Future Work

- Multi-finger and multi-DOF robotic hand
- Force sensing and haptic feedback
- Impedance and adaptive control
- Learning-based gesture mapping
- Hardware-in-the-loop testing
- Vision + force sensor fusion

---

## 🏁 Conclusion

This project demonstrates a **fully integrated mechatronic system** combining perception, modeling, control, embedded implementation, and mechanical design. Both hardware experiments and model-based simulation validate the system’s effectiveness, providing a strong foundation for advanced robotic manipulation and assistive technologies.

---

## 📜 License
