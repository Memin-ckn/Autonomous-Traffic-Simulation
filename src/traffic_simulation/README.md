# Traffic Simulation

A ROS2-based traffic simulation system with a component-based architecture, using pygame for visualization.

## Overview

This project implements a traffic simulation system where all vehicles are fully autonomous. Every car (white or purple) navigates through a road network using sensors, IoT communication, and advanced collision avoidance strategies with predictive ghost-like behavior.

## Features

- Procedurally generated road networks with intersections
- All vehicles are fully autonomous with sensors and IoT
- Sensor simulation (Lidar, Radar) for every vehicle
- IoT communication between all vehicles
- Advanced collision detection and avoidance with predictive "ghost" behavior
- Intelligent path planning and route optimization
- Real-time visualization using Pygame
- Dynamic safety and efficiency scoring system
- Automatic route replanning for collision avoidance

## Project Structure

```
traffic_simulation/
├── launch/           # ROS2 launch files
├── resource/         # ROS2 resource files
├── tests/            # Unit tests and test utilities
├── traffic_simulation/
│   ├── communication/  # IoT and vehicle communication
│   ├── config/         # Configuration files
│   ├── control/        # Vehicle control systems
│   │   └── collision_avoidance_node.py  # Real-time collision avoidance
│   ├── models/         # Visual models and assets
│   ├── planning/       # Path planning and collision avoidance
│   ├── sensors/        # Sensor simulation (Lidar, Radar)
│   ├── unused/         # Deprecated code (for reference)
│   ├── utils/          # Utility functions
│   ├── vehicles/       # Vehicle classes and behaviors
│   ├── visualization/  # Visualization components
│   │   ├── components/ # Modular visualization components
│   │   ├── traffic_visualizer.py  # Main visualization system
│   │   └── vehicle_visualization.py  # Vehicle rendering
```

## Components

### Visualization

- `traffic_visualizer.py`: Main visualization system with real-time display and interaction
- `vehicle_visualization.py`: Handles vehicle rendering and display

### Control

- `collision_avoidance_node.py`: Real-time collision detection and avoidance ROS2 node

### Vehicles

- `vehicle.py`: Base vehicle class and implementations for various vehicle types
- `WhiteCar`: Fully autonomous, sensor-equipped, IoT-enabled vehicle (visually white)
- `SmartCar`: Fully autonomous, sensor-equipped, IoT-enabled vehicle (visually purple)

### Planning

- `collision_avoidance.py`: Planning-based collision avoidance strategies
- `path_planner.py`: Plans optimal paths through the network

### Sensors

- `sensor.py`: Base sensor classes and implementations (Lidar, Radar)

### Communication

- `iot.py`: Vehicle-to-vehicle and vehicle-to-infrastructure communication

## Running the Simulation

### Prerequisites

- ROS2 Humble Hawksbill or newer
- Python 3.x
- Pygame

### Installation

1. Clone the repository to your workspace:
```bash
cd ~/traffic_sim_ws/src
git clone https://github.com/Memin-ckn/Autonomous-Traffic-Simulation.git
```

2. Build the package:
```bash
cd ~/traffic_sim_ws
colcon build --packages-select traffic_simulation
```

3. Source the setup file:
```bash
source ~/traffic_sim_ws/install/setup.bash
```

### Running

The simplest way to run the simulation is using the launch file:

```bash
cd ~/traffic_sim_ws && colcon build --packages-select traffic_simulation && source install/setup.bash && ros2 launch traffic_simulation traffic_sim.launch.py
```

This command will:
1. Navigate to the workspace
2. Build the traffic simulation package
3. Source the setup file
4. Launch the simulation using the launch file

You can also run individual components:

Launch the visualization:
```bash
ros2 run traffic_simulation traffic_visualizer
```

To run the collision avoidance node separately:
```bash
ros2 run traffic_simulation collision_avoidance_node
```

## Usage

### Basic Controls
- Click on intersections to set start and end points
- Press 'S' to randomly select start and end points
- Press 'A' to toggle auto-restart mode (automatically generates new routes)
- Press 'H' to toggle hitboxes and safety zones
- Press 'R' to force the main car to find an alternative route
- Press SPACE to pause/resume simulation
- Press +/- to adjust simulation speed
- Press ESC to exit

### Advanced Features
- Use dropdowns to change grid size and car count
- Watch the safety score, efficiency score, and overall route quality in real-time
- Observe collision avoidance strategies:
  - Slowing down (orange indicator)
  - Yielding (red indicator)
  - Rerouting (blue indicator)
- Monitor turn indicators and safety zones when hitboxes are enabled

## Project Information

- **Version**: 0.2.0
- **Description**: Advanced Autonomous Traffic Simulation System
- **Maintainer**: Mehmet Emin Çakın
- **Email**: mehmetemincakin@gmail.com
- **License**: Apache License 2.0 