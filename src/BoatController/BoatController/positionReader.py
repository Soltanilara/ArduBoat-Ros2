import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
##
class DestinationPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.PositionPublisher_ = self.create_publisher(Float32MultiArray, 'Destination', 10)
        self.listen_for_input()
        #self.leftThrustPublisher_ = self.create_publisher(Int32, 'leftThruster', 10)
        #self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        #self.latest_latitude = 0.0  #
        #self.latest_longitude = 0.0  #
        #self.lock = threading.Lock()  # Ensure thread-safety when accessing latest_input

    def publish_values(self):
        msg = Float32MultiArray()
        msg.data = [self.latest_latitude, self.latest_longitude]
        #msgLeft.data = int(self.latest_leftThrustValue)
        self.PositionPublisher_.publish(msg)
        #self.get_logger().info(f'Publishing: Invalid input.Please enter float values seperated by comma')
        #self.leftThrustPublisher_.publish(msgLeft)
        #self.get_logger().info(f'Published Data')


    def listen_for_input(self):
        while True:
            temp_input1 = input("Enter desired latitude and longitude: ")
            #temp_input2 = input("Enter new value for Left Thruster: ")
            str_values = temp_input1.split(',')
            try:
                self.latest_latitude = float(str_values[0])  # Safely update the latest inputs
                self.latest_longitude = float(str_values[1])
                self.publish_values()

            except ValueError:
                self.get_logger().info(f'Publishing: Invalid input.Please enter float values seperated by comma')

    # def start_input_thread(self):
    #     input_thread = threading.Thread(target=self.listen_for_input)
    #     input_thread.daemon = True
    #     input_thread.start()

def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
