import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
class DestinationPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.SetValuePublisher_ = self.create_publisher(Float32MultiArray, 'VelocityYaw', 10)
        self.listen_for_input()
        self.latest_Vx = 0
        self.latest_Yaw = 0

    def publish_values(self):
        msg = Float32MultiArray()
        msg.data = [self.latest_Vx, self.latest_Yaw]
        self.SetValuePublisher_.publish(msg)


    def listen_for_input(self):
        while True:
            temp_input1 = input("Enter desired velocity and Yaw: ") ##insert checks for values
            #temp_input2 = input("Enter new value for Left Thruster: ")
            str_values = temp_input1.split(',')
            try:
                self.latest_Vx = float(str_values[0])  # Safely update the latest inputs
                self.latest_Yaw = float(str_values[1]) 
                self.latest_Yaw = math.radians(self.latest_Yaw) 
                self.publish_values()
            except ValueError:
                self.get_logger().info(f'Publishing: Invalid input.Please enter float values seperated by comma')


def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
