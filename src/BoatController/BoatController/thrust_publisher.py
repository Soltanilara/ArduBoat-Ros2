"""
This is a ROS2 publisher node to read user input values and publish the values to topic /Thrusters.
The user needs to input right and left thruster values(PWM Signal) to the boat, which must range between 1000 to 2000. 
A value of 1500 is neutral, below 1500 backward rotation and above 1500 is forward rotating.
Note these are expected trim values, 
actual values may differ by +- 10. 

The program is multi-threaded to allow user input value at their preference. This means that, 
the publisher will always publish the last user value to the topic
"""
"""
Filename: thrust_Publisher.py
Description: ROS2 publisher for sending individual thruster value to ardupilot
Author: Dinesh Kumar
Date: 2024-04
License: MIT License
"""

#Standard Library imports for ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys
#Library import of threading
import threading

# The following class, creates and handles the publisher node. 
class ThreadedInputPublisher(Node):  
    def __init__(self):
        """ set up the publisher """
        super().__init__('threaded_input_publisher')
        self.running = True
        self.ThrustPublisher_ = self.create_publisher(Int32MultiArray, 'Thrusters', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish at 100 Hz
        self.latest_rightThrustValue = 1500  # Default neutral values
        self.latest_leftThrustValue = 1500  
        self.lock = threading.Lock()  # Ensure thread-safety when accessing latest_input

    def timer_callback(self): 
        """ locks the thread to create message and pubslish to topic """
        with self.lock:
            msg = Int32MultiArray()
            msg.data = [self.latest_rightThrustValue, self.latest_leftThrustValue]
            self.ThrustPublisher_.publish(msg)

    def listen_for_input(self): #Always on loop, waiting for user input
        """ Waits for user input thruster values """
        while True:
            try:
                temp_input1 = input("Enter new value for Right Thruster: ")
                temp_input2 = input("Enter new value for Left Thruster: ")
                temp_input1 = int(temp_input1)
                temp_input2 = int(temp_input2)
                if(temp_input1 > 2000 or temp_input1 < 1000 or temp_input2 > 2000 or temp_input2 < 1000):
                    self.get_logger().info(f'Publishing: Invalid input. Range must be within 1000 to 2000')
                else:
                    with self.lock:
                        self.latest_rightThrustValue = temp_input1  # Safely update the latest input
                        self.latest_leftThrustValue = temp_input2  # Safely update the latest input
            except KeyboardInterrupt:
                self.signal_handler(None, None)

    def start_input_thread(self): 
        """ Handle Threads"""
        input_thread = threading.Thread(target=self.listen_for_input)
        input_thread.daemon = True
        input_thread.start()

def main(args=None): #Main Loop 
    rclpy.init(args=args)
    node = ThreadedInputPublisher()
    node.start_input_thread()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
