# README – Moving Base Auto-Alignment System

## Overview
- We are building a system where a **ground robot** (an mecanum base) automatically moves itself **underneath a flying tailsitter VTOL**.
- The quadcopter is **not controlled automatically** — it is moved manually (Use this model https://app.gazebosim.org/alipoorhoseini/fuel/models/CERBERUS_M100_SENSOR_CONFIG_1 ).  
- The ground robot **is controlled automatically** — it uses a **camera pointing upward** to see LED markers on the quadcopter, and it drives itself to stay directly under it.

Think of it as:  
**“The drone hovers/moves around. The robot chases it and parks perfectly below it.”**

---

## Main Idea
The quadcopter has **two IR LEDs**, one on each ends.  
The base has a **camera pointing straight up**.

The camera sees the LEDs as **bright dots**.  
From those dots, the robot figures out:

- Where the tailsitter/quadcopter is (in pixel space)
- How far off-center it is
- Which direction it must move to get underneath it

**The goal:**  
Make the LED dots sit in the **center of the camera image** → meaning the robot is directly underneath the tailsitter.

---

## Components

### 1. Quadcopter UAV
- Acts as a “target in the sky”
- Controlled manually by arrow keys in simulation
- Has 2 LED markers on ends

### 2. Mecanum wheel Base
- Moves freely in any direction (holonomic)
- Has a camera pointing upward
- Receives `/cmd_vel` commands to move

### 3. Upward-Facing Camera
- Captures video of the UAV from below
- Detects the bright LED markers
- Processes the image to find the LED centroid

### 4. LED Tracker Node (ROS2)
This node:
1. Reads the camera image  
2. Detects the LEDs  
3. Calculates how far they are from the image center  
4. Converts that error into robot movement commands  
5. Publishes `/cmd_vel` for the mecanum wheel base

This is effectively **vision-based alignment**.

---

## How It Works (Simple Explanation)

1. **Camera sees the LEDs**
2. System finds their position in the image
3. If LEDs are not centered → robot moves to center them
4. When LEDs stay centered → robot is directly under the UAV

It works like this:

- If LEDs appear to the **right**, robot moves **right**  
- If LEDs appear **forward**, robot moves **forward**  
- And so on…

This mimics how a person would position themselves under a hanging lamp by looking up.

---

## Simulation Setup (What We Build)
We will simulate:

- A **UAV model (https://app.gazebosim.org/alipoorhoseini/fuel/models/CERBERUS_M100_SENSOR_CONFIG_1)** with LED markers  
- A **mecanum 4-wheel robot** with an upward camera  
- ROS2 nodes for:
  - UAV teleop (arrow keys)
  - Camera streaming
  - LED detection & tracking
  - Base control

Everything happens inside **ROS2 + Gazebo**.

---

## Why This Setup?
- Simple to implement  
- Easy to visualize and debug  
- Realistic to how an actual UAV or VTOL docking/catching system works  
- Sets foundation for future landing/catching logic

---

## End Goal
The mecanum wheel base should automatically:

- Track the UAV 
- Follow it as it moves  
- Stay positioned directly underneath it  
- Maintain stable alignment even as the drone drifts  

Later, this can be expanded to:

- Vertical descent synchronization  
- Safe capture/docking  
- Precision landing  

---
