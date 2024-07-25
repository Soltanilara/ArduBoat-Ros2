# ROS2 ArduBoat Control

The following are ROS2 nodes for different control methods developed for the USV paper titled "XXXXXX" published in IEEE "XXXX". Outlined here are setup instructions for running the ROS2 nodes, setup procedures, and setup on simulation. The framework is tested on ROS2 Humble on Ubuntu 22.04 LTS running on Jetson Orin Nano. 

# Table of Contents

- [Project Description](#project-description)
- [Installation](#installation)
- [Usage](#usage)
- [Connection Setup Instructions](#connection-setup-instructions)
- [Control Mode Explanation](#control-mode-explanation)
- [Simulation Setup](#simulation-setup)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

# Project Description

This project includes ROS2 nodes designed for various control methods applied to an Unmanned Surface Vehicle (USV). The nodes were developed as part of the research detailed in the paper titled "XXXXXX", published in IEEE "XXXX".

# Installation

Follow these steps to install the necessary dependencies and set up the project:

1. Clone the repository:
    ```sh
    git clone https://github.com/dskuma/ArduBoat-Ros2.git
    cd ros2-arduboat-control
    ```

2. Install ROS2 (Humble) for your target:
    Follow the official ROS2 installation guide: [ROS2 Installation](https://docs.ros.org/en/humble/Installation.html)

3. Source the ROS2 setup:
    ``` sh
    source /opt/ros/humble/setup.bash ##Need to do this every terminal
    ``` 
    or add it permanently to your bashrc using 
    ``` sh
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc ##Do it just once to avoid sourcing every terminal
    ```

4. Install Pymavlink and other dependencies:
    ```sh
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions
    pip install pymavlink
    ```

5. Build the workspace:
    ```sh
    colcon build --symlink-install
    ```
6. Download and install QGroundControl: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu
    
# Connection Setup Instructions

Detailed setup instructions for running the ROS2 nodes are as follows:

1. **Hardware Setup**:
   - Connect USB port of Companion Computer to CubeOrange through its USB port. Refer Image. Connection determines wether to use ACM or USB. 

2. **Software Setup**:
   - Ensure all necessary drivers and software packages are installed. Check for requirements based on your choice of companion computer.

3. **ROS2 Node Configuration**:
   - Modify the configuration files as needed to match your setup.


# Usage

1. Source the ROS2 setup script:
    ```sh
    source ~/ArduBoat-Ros2/install/setup.bash ##Need to do this every terminal
    ```
    OR
    ```sh
    echo 'source ~/ArduBoat-Ros2/install/setup.bash' >> ~/.bashrc ##Do it just once to avoid sourcing every terminal
    ```
2. Source the workspace:
    ```sh
    source install/setup.bash
    ```

3. Run the ROS2 nodes:
    ```sh
    ros2 run BoatController node_name
    ```

# Control Mode Explanation
The following section explains how to use the control modes described in sectionXX of paper XX. All commands to run publisher and subscriber are presented here. Before running commands, please ensure the parameter file is modified as explained under each subsection. 

Assuming the following is running on a companion computer on-board the USV, the user needs to setup a local network over wifi and ssh into the computer to run the following codes. Currently the publisher and subscriber all run within the companion computer. 

The connection url determines how the companion computer is talking to the Ardupilot. For physical testing, ensure to use __"/dev/ttyACMx"__, __"/dev/ttyUSBx"__ where x determines which port number. Typically these are __"/dev/ttyACM0"__ or __"/dev/ttyUSB0"__ on Linux based systems.

Future works involves modifying it to use publisher/subscriber over network.
## **1. Direct Thruster Control Mode**
## Warning

> **⚠️ Attention!**
>
> **The developers of this code, understand and advise that this method be only used if the development team is confident of its works and well aware of their goals. This method is generally advised against if you are a newbie developer with ArduPilot/pymavlink as most safety checks are overridden.**

- This mode is void of any control loop of ardupilot and directly commands the left and right thruster by mapping each  
       thruster to a channel on the RC. Under guided mode, this value is sent from the companion computer instead of remote control. 

- Assuming motors are connected to servos 1 and 3 of CubeOrange please ensure your servo outputs are mapped as follows:
    - Servo1_Function: RCIN_1
    - Servo2_Function: RCIN_4

    User can feel free to change it to any other channel. If these channels are changed, please remember to reflect the changes on the code base.

## **2. Waypoint Control** 
- This mode commands a lattitude/longitude to the Ardupilot. ASsuming motors are connected to servos 1 and 3 of CubeOrange please ensure your servo outputs are mapped as follows:
    - Servo1_Function: Thruster Left
    - Servo2_Function: Thruster Right

- Open one terminal, ensure the package is sourced, and start the publisher
    ```sh
    ros2 run BoatController positionPublisher 
    ```

- Open second terminal, ensure the package is sourced, and start the subscriber
    ```sh
    ros2 run BoatController waypointSubscriber 
    ```
- On prompting, enter desired Lattitude & Longitude.

## **3. Velocity and Yaw Control** 
- This mode commands a velocity and Yaw(degrees) to the Ardupilot in body frame. Assuming motors are connected to servos 1 and 3 of CubeOrange please ensure your servo outputs are mapped as follows:
    - Servo1_Function: Thruster Left
    - Servo2_Function: Thruster Right

- Open one terminal, ensure the package is sourced, and start the publisher
    ```sh
    ros2 run BoatController velocityYawPublisher
    ```

- Open second terminal, ensure the package is sourced, and start the subscriber
    ```sh
    ros2 run BoatController velocityYawSetter 
    ```
- On prompting, enter desired Velocity & Yaw(degrees).

# Simulation Setup
Simulation setup allows for the user to test the whole codestack on the computer, to verify its working and testing before deploying it on the actual system. This simulation stack involves, simulating arduRover at a particular location, communication with an companion computer and status monitoring on QGroundControl. This setup simulates the USV deployment. 

**⚠️ Attention!**
>This setup however does not simulate collision with terrain, vehicle dynamics and 
  environmental factors.
>

The code comments out the URL strings as needed. When using simulation ensure to use "tcp:localhost:5762" as this would allow for companion computer connection alonside QGroundControl on simulation. 

To set up the simulation environment on Ubuntu 22.04 LTS x86 system, follow these steps:

1. **Install Simulation Tools**:
   - Install Simulation Tools using https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html. 

   ###  Note: Though the article mentions it is tested only until 18.04, the following works fine on Ubuntu 22.04. 

2. **Configure Simulation**:
   - This ensure the ardupilot tools are properly sourced. 
   ``` sh
    echo 'source /home/dinesh/ardupilot/Tools/completion/completion.bash' >> ~./bashrc
     ```
   - The simulation start locations can be edited and if you would like to add your own location, please edit Tools/autotest/locations.txt within ardupilot's home directory.
 
3. **Run Simulation**:
   - Open Simulation 
   ```sh
   cd ardupilot
   sim_vehicle.py -v Rover --console --map  -L <Your Location>
    ```
   - Open QGroundControl on Ubuntu. The app should automatically connect to Ardupilot running. 

