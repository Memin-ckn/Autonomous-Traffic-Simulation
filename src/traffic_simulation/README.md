# Traffic Simulation

A ROS2-based traffic simulation system with a component-based architecture, using pygame for visualization.

## Overview

This project implements a traffic simulation system where vehicles navigate through a road network while avoiding collisions. The simulation includes various vehicle types with different behaviors, sensors, and collision avoidance strategies.

## Features

- Procedurally generated road networks with intersections
- Multiple vehicle types with different behaviors
- Sensor simulation (Lidar, Radar)
- IoT communication between vehicles
- Collision detection and avoidance
- Path planning and route optimization
- Real-time visualization using Pygame

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
│   │   ├── gui.py      # Main GUI
│   │   ├── vehicle_visualization.py  # Vehicle rendering
│   │   ├── simple_visualizer.py       # Original visualizer
│   │   ├── simple_visualizer_updated.py # Updated version
│   │   └── refactored_visualizer.py   # Fully refactored version
```

## Components

### Visualization

- `renderer.py`: Handles all drawing operations
- `world_model.py`: Manages the road network and pathfinding
- `collision_detector.py`: Handles collision detection and prediction
- `ui_manager.py`: Manages user interface elements
- `vehicle_visualization.py`: Handles vehicle rendering and display

### Control

- `collision_avoidance_node.py`: Real-time collision detection and avoidance ROS2 node

### Vehicles

- `vehicle.py`: Base vehicle class and implementations for various vehicle types
- `WhiteCar`: Standard vehicle with basic routing
- `SmartCar`: Advanced vehicle with collision avoidance

### Planning

- `collision_avoidance.py`: Planning-based collision avoidance strategies
- `path_planner.py`: Plans optimal paths through the network
- `route_planner.py`: High-level route planning

### Sensors

- `sensor.py`: Base sensor classes and implementations (Lidar, Radar)

### Communication

- `iot.py`: Vehicle-to-vehicle and vehicle-to-infrastructure communication

## Running the Simulation

### Prerequisites

- ROS2 Jazzy Jalisco
- Python 3.x
- Pygame

### Installation

1. Clone the repository to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-repo/traffic_simulation.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select traffic_simulation
```

3. Source the setup file:
```bash
source ~/ros2_ws/install/setup.bash
```

### Running

Launch the GUI:
```bash
ros2 run traffic_simulation gui
```

Or use one of the visualizers:
```bash
# Original visualizer
ros2 run traffic_simulation simple_visualizer

# Updated visualizer with components
ros2 run traffic_simulation simple_visualizer_updated

# Fully refactored visualizer
ros2 run traffic_simulation refactored_visualizer
```

To run the collision avoidance node separately:
```bash
ros2 run traffic_simulation collision_avoidance_node
```

## Usage

- Click on intersections to set start and end points
- Press 'S' to randomly select start and end points
- Press 'H' to toggle hitboxes for collision detection
- Press 'R' to force the main car to find an alternative route
- Press 'P' to check the route and preventatively reroute
- Use dropdowns to change grid size and car count
- Press SPACE to pause/resume simulation
- Press +/- to adjust simulation speed
- Press ESC to exit

## Project Information

- **Version**: 0.1.0
- **Description**: Bitirme Projesi
- **Maintainer**: Mehmet Emin Çakın
- **Email**: mehmetemincakin@gmail.com
- **License**: Apache License 2.0 