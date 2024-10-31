import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray
from BoatController import mavlink_utilities

"""
This is a ROS2 Subscriber node to command velocity and heading in 
body frame through mavlink into Ardupilot. 
Velocity and Yaw is read of topic /VelocityYaw. 
"""
"""
Filename: velocityYaw_subscriber.py
Description: ROS2 subscriber for commanding velocity and heading setpoints.
Author: Dinesh Kumar
Date: 2024-04
License: MIT License
"""

#Class for subscriber node
class VelocityYawCommander(Node): 
    def __init__(self, boat, boot_time):
        """ Setup the Subscriber """
        super().__init__('VelocityYawCommander')
        self.running = True
        self.Vx = 0         # initial velocity
        self.Yaw_rads = 0   # initial heading
        self.boot_time = boot_time
        self.boat = boat
        # Subscriber setup
        self.create_subscription(Float32MultiArray, 'VelocityYaw', self.subscriber_callback, 10) 
        self.timer = self.create_timer(0.01, self.publish_velocityYaw_values) #100Hz
    
    def subscriber_callback(self, msg): 
        """ This function reads data from the topic """
        self.Vx = msg.data[0]
        self.Yaw_rads = msg.data[1]
        self.get_logger().info(f'{msg.data[0]}, {msg.data[1]}') 
    
    def publish_velocityYaw_values(self): # 
        """ This function sends commands to Ardupilot """
        mavlink_utilities.setX_velocity_and_yaw(self.boat,self.boot_time,self.Vx,self.Yaw_rads)

def main(args=None):
    try:
            rclpy.init(args=args)
            url = "tcp:localhost:5762" 
            """
            url changes according to use case. If in simulation 
            use "tcp:localhost:5762" or udp:localhost:14550. 
            If on physical boat use either "/dev/ttyUSBx" or "/dev/ttyACMx" or 
            "/dev/ttyTHSx"
            """ 
    # Setup MAVLink connection and Thruster Controller
            boat = mavlink_utilities.setup_connection(url)
            boot_time = time.time()
            node = VelocityYawCommander(boat, boot_time)
            rclpy.spin(node)
            mavlink_utilities.disarm_vehicle(boat)
            node.destroy_node()
            rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
            mavlink_utilities.disarm_vehicle(boat)
            node.destroy_node()
                

if __name__ == '__main__':
    main()
