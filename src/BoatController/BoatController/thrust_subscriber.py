"""
This is a ROS2 Subscriber node to implement values from /Thrusters to the thrusters on the boat. 
The subscriber reads from the topic, and sends a RC Overide command using mavlink.

The program is multi-threaded to allow user input value at their preference. This means that, 
the subscriber will always output the last value on the topic. Read comments/ Readme for available connection strings. 
"""

"""
Filename: thrust_subscriber.py
Description: ROS2 subscribing for sending individual thruster value to ardupilot
Author: Dinesh Kumar
Date: 2024-04
License: MIT License
"""
#Standard Imports for ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import sys
from BoatController import mavlink_utilities

class ThrusterController(Node):
    def __init__(self,boat=None):
        """ Setup Subscriber"""
        super().__init__('thruster_controller')
        self.thruster_values = [1500,1500]
        self.boat = boat
        self.create_subscription(Int32MultiArray, '/Thrusters', self.thruster_callback, 10) #Subscriber
    
    def signal_handler(self, sig, frame):
        """ Signal handler for shutting down"""
        self.get_logger().info('Shutdown')
        #self.running = False
        self.destroy_node()
        sys.exit()

    def thruster_callback(self, msg):
        """ Sends thruster values from topic to Boat """
        self.thruster_values  = msg.data
        self.get_logger().info(f'Right Thruster: {self.thruster_values[0]}, Left Thruster: {self.thruster_values[1]}') #print value to screen
        mavlink_utilities.control_rc_channels(self.boat, (msg.data[0]), (msg.data[1]) )
        #self.publish_thruster_values(msg)
            

def main(args=None): 
    rclpy.init(args=args)
    url = "tcp:localhost:5762" #Change according to purpose. Read below or README for more information
    """
    url changes according to use case. If in simulation use "tcp:localhost:5762" or udp:localhost:14550. If on physical boat use either "/dev/ttyUSBx" or "/dev/ttyACMx" or "/dev/ttyTHSx"
    
    """ 
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
