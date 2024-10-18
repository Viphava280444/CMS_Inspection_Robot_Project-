import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String  
import time  
import serial  

# Define a ROS2 Node for communicating with an Arduino via serial
class ArduinoCommunicatorNode(Node):
    def __init__(self):
        super().__init__('arduino_communicator')
        
        # Initialize serial communication with Arduino
        # Replace '/dev/ttyACM0' with the correct serial port and '115200' with the appropriate baud rate for your Arduino
        self.serial = serial.Serial('/dev/ttyACM0', 115200)
        
        # Subscription to listen for messages on the 'robot_parameters' topic
        self.subscription = self.create_subscription(String, 'robot_parameters', self.parameters_callback, 1)
        
        # Publisher to send messages to the 'arduino_output' topic
        self.publisher_ = self.create_publisher(String, 'arduino_output', 10)

    # Callback function triggered when a message is received on the 'robot_parameters' topic
    def parameters_callback(self, msg):
        command = msg.data  
        self.send_command(command)  # Send the command to Arduino

    # Function to send a command to the Arduino via serial
    def send_command(self, command):
        self.serial.write(command.encode("utf-8"))  
        data = self.serial.readline().strip()  
        
        # If data is received from Arduino, publish it to the 'arduino_output' topic
        if data:
            self.publish_output(data.decode("utf-8"))  

    # Function to publish the received data to the 'arduino_output' topic
    def publish_output(self, data):
        output_msg = String()
        output_msg.data = data  
        self.publisher_.publish(output_msg)  # Publish the output message

    # Main loop to keep the node running
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)  # Process ROS2 callbacks and handle incoming messages

# Main function to initialize and run the ROS2 node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = ArduinoCommunicatorNode()  # Create an instance of the ArduinoCommunicatorNode
    node.run()  # Run the node
    rclpy.shutdown()  # Shutdown ROS2 when done

# Entry point for the script
if __name__ == '__main__':
    main()  
