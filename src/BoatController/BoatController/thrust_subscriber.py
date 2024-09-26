"""
This is a ROS2 Subscriber node to implement values from /Thrusters to the thrusters on the boat. 
The subscriber reads from the topic, and simulates a RC Overide using mavlink.
This program is not compatible with the simulation environment, as it is not 
possible to simulate motors as RCIN on Ardupilot simulation
The program is multi-threaded to allow user input value at their preference. This means that, 
the subscriber will always output the last value on the topic. Read comments/ Readme for available connection strings. 
"""

"""
Filename: thrustPublisher.py
Description: ROS2 subscribing for sending individual thruster value to ardupilot
Author: Dinesh Kumar
Date: 2024-04
"""
#Standard Imports for ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import sys

#Importing Class for handling communication to Ardupilot
from BoatController import mavlink_utilities

class ThrusterController(Node):
    def __init__(self,boat=None):
        super().__init__('thruster_controller')
        # Initialize variables
        self.thruster_values = [1500,1500]
        # boat communication setup
        self.boat = boat
        # Subscribers
        self.create_subscription(Int32MultiArray, '/Thrusters', self.thruster_callback, 10) #change to thruster callback
    
    def signal_handler(self, sig, frame):
        self.get_logger().info('Shutdown')
        #self.running = False
        self.destroy_node()
        sys.exit()

    def thruster_callback(self, msg):
        #mavlink_utilities.arm_vehicle(self.boat) #Arm Vehicle
        self.thruster_values  = msg.data
        self.get_logger().info(f'Right Thruster: {self.thruster_values[0]}, Left Thruster: {self.thruster_values[1]}') #print value to screen
        mavlink_utilities.control_rc_channels(self.boat, (msg.data[0]), (msg.data[1]) )
        #self.publish_thruster_values(msg)
       
    def publish_thruster_values(self,msg): #Function to command thruster. 
        """
        WARNING: ALL SAFETY CHECKS ARE ABANDONED. PROCEED AT YOUR OWN RISK
        """
       
            

def main(args=None): 
    rclpy.init(args=args)
    # Create a standalone Node to access parameters
    #url = '/dev/ttyACM0' # dev/ttyUSBx or /dev/ttyACMx or /dev/ttyTHSx. Replace x with number. Refer Readme.md for setup
    url = "tcp:localhost:5762"
    #Setup MAVLink connection and Thruster Controller
    boat = mavlink_utilities.setup_connection(url) #creating the boat class for connection
    thruster_controller = ThrusterController(boat) #creating the subscriber node
    rclpy.spin(thruster_controller)
    # Executor setup
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(thruster_controller)
    executor.spin()
        # Cleanup
    #mavlink_utilities.disarm_vehicle(boat)
    thruster_controller.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()





# def main(args=None):
#     #setting up mavlink communications
#     url = "/dev/ttyACM2"
#     boat = mavlink_utilities.setup_connection(url)
#     rclpy.init(args=args)
#     thruster_controller = ThrusterController(boat)
#     executor = SingleThreadedExecutor()
#     executor.add_node(thruster_controller)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         thruster_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
