# 🤖 Robotics Project

## 📖 Overview
This project explores the control and simulation of a **UR10e robotic arm** with six degrees of freedom. The goal is to develop and simulate control algorithms for tracking and interacting with a moving sphere using MATLAB and the Robotics Toolbox.

---

## 🧩 Simulation 1: Tracking a Moving Sphere

### 🎯 Goals
- Design a control loop to track a moving sphere.
- Maintain a specified orientation and distance relative to the sphere.

### ✨ Features
- **Robotic Arm Control**:
  - The robotic arm tracks the sphere’s movement using velocity commands for its joints.
- **Mathematical Modeling**:
  - Homogeneous transformation matrices to compute desired position and orientation.
  - Control laws based on position and orientation errors.
- **Simulation**:
  - Runs for 10 seconds with a 2 ms control cycle.
  - Visualizes joint angles, velocities, and accelerations.

### 🏆 Results
- The arm successfully tracks the sphere with minimal errors.
- Errors in position and orientation decrease over time, achieving accurate tracking.

---

## 🛠️ Simulation 2: Grasping the Moving Sphere

### 🎯 Goals
- Extend the tracking control to enable the robotic arm to grasp the moving sphere using a gripper.
- Approach the sphere slowly to avoid contact with the surface of the slide.

### ✨ Features
- **Gripper Integration**:
  - Adds a gripper to the arm’s end effector for grasping.
- **Dynamic Positioning**:
  - Moves the gripper closer to the sphere while maintaining orientation.
- **Simulation**:
  - Includes a 1-second delay to secure the grasp once in position.
  - Visualizes joint behavior during approach and grasp.

### 🏆 Results
- The robotic arm accurately approaches and grasps the sphere.
- Position and orientation errors remain low, even during the grasping process.

---

## 📂 Repository Contents
- **Report**: Detailed analysis and findings in [Robotics Project Report](./Robotics.pdf).
- **Code**: MATLAB scripts for simulating the robotic arm’s motion and control.
  - Includes scripts for both tracking and grasping simulations.

---

Thank you for exploring this project! 🚀 Feel free to raise issues or contribute to improve the repository. 😊
