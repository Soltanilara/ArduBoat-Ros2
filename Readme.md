# ROS2 ArduBoat Control

The following are ROS2 nodes for different control methods developed for the USV paper titled "XXXXXX" published in IEEE "XXXX". Outlined here are setup instructions for running the ROS2 nodes, setup procedures, and setup on simulation.

## Table of Contents

- [Project Description](#project-description)
- [Installation](#installation)
- [Usage](#usage)
- [Setup Instructions](#setup-instructions)
- [Simulation Setup](#simulation-setup)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

## Project Description

This project includes ROS2 nodes designed for various control methods applied to an Unmanned Surface Vehicle (USV). The nodes were developed as part of the research detailed in the paper titled "XXXXXX", published in IEEE "XXXX".

## Installation

Follow these steps to install the necessary dependencies and set up the project:

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/ros2-arduboat-control.git
    cd ros2-arduboat-control
    ```

2. Install ROS2 (e.g., Foxy, Galactic):
    Follow the official ROS2 installation guide: [ROS2 Installation](https://docs.ros.org/en/foxy/Installation.html)

3. Install necessary dependencies:
    ```sh
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions
    ```

4. Build the workspace:
    ```sh
    colcon build
    ```

## Usage

1. Source the ROS2 setup script:
    ```sh
    source /opt/ros/foxy/setup.bash
    ```

2. Source the workspace:
    ```sh
    source install/setup.bash
    ```

3. Run the ROS2 nodes:
    ```sh
    ros2 run package_name node_name
    ```

## Setup Instructions

Detailed setup instructions for running the ROS2 nodes are as follows:

1. **Hardware Setup**:
   - Connect your hardware as described in the hardware setup guide.

2. **Software Setup**:
   - Ensure all necessary drivers and software packages are installed.

3. **ROS2 Node Configuration**:
   - Modify the configuration files as needed to match your setup.

## Simulation Setup

To set up the simulation environment, follow these steps:

1. **Install Simulation Tools**:
   - Install Gazebo or other simulation tools compatible with ROS2.

2. **Configure Simulation**:
   - Modify the simulation configuration files to match your desired simulation setup.

3. **Run Simulation**:
   ```sh
   ros2 launch package_name simulation_launch_file.launch.py
