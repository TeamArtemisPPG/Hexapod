# 🕷️ Hexapod — Autonomous AI Hexapod Robot Rover

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-informational?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo_Harmonic-orange)](https://gazebosim.org/)
[![NVIDIA NIM](https://img.shields.io/badge/AI_Brain-NVIDIA_NIM-76B900?logo=nvidia)](https://build.nvidia.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

An advanced 6-legged autonomous hexapod robot platform built with **ROS 2 Jazzy**, integrating physical hardware bridges, Gazebo Harmonic simulation, SLAM & Nav2 navigation, and an autonomous AI brain agent powered by **NVIDIA NIM Vision-Language Models** (LLM/VLM) with real-time tool calling and voice interaction (Codename: **Zoe**).

---

## 🌟 Key Features

1. **Autonomous AI Brain ("Zoe")**
   - Integrates NVIDIA NIM API (`meta/llama-3.2-11b-vision-instruct`, `meta/llama-3.1-70b-instruct`) with multi-step tool-calling agentic loop.
   - Dynamic tool dispatching for autonomous movement, waypoint navigation, sensor audits, exploration, and pan/tilt camera inspection.
   - Secure credential management keeping API keys in local `.brain_secrets` or environment variables.

2. **Multimodal Perception & Sensor Bridge**
   - Aggregates 2D LiDAR scan data (360° partitioned into front, left, right, rear sectors).
   - Real-time vision-based object detection and bounding box tracking (`sensor_bridge_node`).
   - IMU orientation & pitch/roll stabilization (`imu_stabalizer`).

3. **Locomotion & Inverse Kinematics**
   - 18-DOF hexapod kinematics with 3-DOF per leg (Coxa, Femur, Tibia).
   - Dynamic tripod gait generator (`tripod_gait.py`) and inverse kinematics controllers (`hexapod_ik_controller.py`).
   - Robotic arm IK control & gripper integration (`arm_ik_commander.py`).

4. **Hardware & Simulation Ready**
   - **Hardware**: Direct serial bridge communicating with onboard ESP32 microcontrollers (`esp.py`, `test_esp.py`).
   - **Simulation**: Gazebo Harmonic (`gz_sim`) worlds and URDF models with physics plugins.
   - **Navigation**: Full SLAM Toolbox mapping and Nav2 autonomous path planning with frontier exploration.

5. **Voice Interface**
   - Push-to-talk voice bridge node (`voice_bridge_node.py` and `test_voice_talk.py`).
   - Wake-word filtering (`"Hey Zoe"`) with STT audio transcription and TTS speech feedback.

---

## 📁 Repository Structure

```
.
├── src/
│   ├── rover/                    # Primary ROS 2 hexapod package
│   │   ├── config/               # Nav2, SLAM, controller & brain configurations
│   │   ├── launch/               # Gazebo, RViz, Navigation & Autonomous launch files
│   │   ├── meshes/ & meshes1/    # STL 3D models for hexapod body and legs
│   │   ├── rover/                # Python nodes (Brain, Gait, IK, Sensors, Hardware Bridge)
│   │   ├── urdf/                 # Robot URDF descriptions
│   │   └── worlds/               # Gazebo simulation worlds (SDF)
│   ├── rover_interfaces/         # Custom ROS 2 services (ExecuteTool, SensorSnapshot)
│   └── rplidar_ros/              # Slamtec RPLidar ROS 2 driver package
├── autonomous_brain_plan.md      # Autonomous brain architecture specification
├── .brain_secrets.example        # Example template for NVIDIA API key (DO NOT commit secrets)
├── run_brain.sh                  # One-click startup script for the autonomous brain
├── test_brain_e2e.py             # End-to-end integration test runner
└── test_voice_talk.py            # Interactive voice test utility
```

---

## 🚀 Getting Started

### 1. Prerequisites
- **Ubuntu 24.04 LTS** with **ROS 2 Jazzy Jalisco**
- **Gazebo Harmonic** (`ros-jazzy-ros-gz`)
- **Nav2 & SLAM Toolbox**:
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
  ```
- **Python Dependencies**:
  ```bash
  pip install openai SpeechRecognition pyserial
  sudo apt install speech-dispatcher alsa-utils
  ```

### 2. Configure NVIDIA API Key (Protected)
> [!IMPORTANT]
> Never hardcode or commit your API keys. Use `.brain_secrets`:

```bash
cp .brain_secrets.example .brain_secrets
```
Edit `.brain_secrets` and insert your NVIDIA NIM key:
```env
NVIDIA_API_KEY=nvapi-your_api_key_here
```
*(Or export `export NVIDIA_API_KEY="nvapi-..."` in your shell).*

### 3. Build the Workspace
```bash
cd /path/to/gz_hex
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 Running the System

### Simulation (Gazebo & RViz)
```bash
ros2 launch rover 3_gazebo.launch.py
```

### SLAM Mapping & Navigation
```bash
# Mapping mode:
ros2 launch rover mapping.launch.py

# Autonomous navigation with Nav2:
ros2 launch rover sim_navigation.launch.py
```

### Autonomous AI Brain Stack
Launch the AI brain agent:
```bash
./run_brain.sh
```
Or send high-level mission queries via ROS 2 topics:
```bash
ros2 topic pub --once /brain/query std_msgs/msg/String "{data: 'Hey Zoe, rotate 45 degrees and check your front lidar'}"
```

### Push-to-Talk Voice Tester
```bash
python3 test_voice_talk.py
```

---

## 🔒 Security & Privacy

- All sensitive keys (`.brain_secrets`, `.env*`, `*.key`) are excluded via `.gitignore`.
- Run scripts safely load secrets dynamically without exposing tokens to git history.
