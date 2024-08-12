"""
This is a ROS2 Subscriber node to command velocity and heading in body frame through mavlink into Ardupilot. Velocity and Yaw is read of topic /VelocityYaw. 
"""

"""
Filename: velocityYawControl.py
Description: ROS2 subscriber for commanding velocity and heading setpoints.
Author: Dinesh Kumar
Date: 2024-04
"""
#Standard Library import for ROS2
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

#Class import for Mavlink Handling
from BoatController import mavlink_utilities

class VelocityYawCommander(Node): #Class for subscriber node
    def __init__(self,boat,boot_time):
        super().__init__('setVelocityYaw_command')
        #initialize parameters
        self.Vx = 0 #initial velocity
        self.Yaw_rads = 0 #initial heading
        self.boot_time = boot_time
        self.boat = boat

        # Subscriber
        self.create_subscription(Float32MultiArray, 'VelocityYaw', self.subscriber_callback, 10) 
        self.timer = self.create_timer(0.01, self.publish_velocityYaw_values) #100Hz
        

    def subscriber_callback(self, msg): #This function reads data off the topic
        self.Vx = msg.data[0]
        self.Yaw_rads = msg.data[1]
        self.get_logger().info(f'{msg.data[0]}, {msg.data[1]}') #debug purposes
    
    def publish_velocityYaw_values(self): #This function sends commands to Ardupilot
        mavlink_utilities.setX_velocity_and_yaw(self.boat,self.boot_time,self.Vx,self.Yaw_rads)
    
    def printValue(self): #Helper Function. Needs evaluation to determine if required
        local_position_ned = self.boat.recv_match(type='LOCAL_POSITION_NED', blocking=True) #move to utility function
        if local_position_ned:
            vx = local_position_ned.vx  # Velocity in X (m/s) - North
            print(f"Velocity (NED): vx: {vx} m/s)")   
        
        # Specifically wait for ATTITUDE message
        attitude = self.boat.recv_match(type='ATTITUDE', blocking=True) ##move to utility function
        if attitude:
            yaw = attitude.yaw  # Yaw in radians
            print(f"Yaw: {yaw} radians")


def main(args=None):
    rclpy.init(args=args)
    url = "tcp:localhost:5762" 
    """
    url changes according to use case. If in simulation use "tcp:localhost:5762" or udp:localhost:14550. 
    If on physical boat use either "/dev/ttyUSBx" or "/dev/ttyACMx" or "/dev/ttyTHSx"
    """ 
    # Setup MAVLink connection and Thruster Controller
    boat = mavlink_utilities.setup_connection(url)
    boot_time = time.time()
    velocityYawSetter = VelocityYawCommander(boat,boot_time)
    rclpy.spin(velocityYawSetter)
    mavlink_utilities.disarm_vehicle(boat)
    velocityYawSetter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
