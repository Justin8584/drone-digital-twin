# Digital Twin Drone Simulation System

A comprehensive drone simulation environment for wildlife protection research, combining PX4 autopilot, ROS2, Gazebo physics simulation, and Unity visualization.

## Project Overview

This project creates a digital twin simulation system for autonomous drones designed for garden wildlife protection. It features:

- **Realistic flight dynamics** using PX4 SITL
- **Physics-based simulation** with Gazebo
- **3D visualization** in Unity
- **Wildlife detection** algorithms
- **Autonomous mission planning**

## Quick Start

See [docs/setup/quick_start.md](docs/setup/quick_start.md) for installation instructions.

## Architecture

![System Architecture](docs/images/architecture.png)

## Components

1. **Flight Control** (PX4 SITL)
2. **Environment Simulation** (Gazebo)
3. **Wildlife Detection** (ROS2 + OpenCV)
4. **Mission Control** (ROS2 State Machine)
5. **Visualization** (Unity 3D)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
