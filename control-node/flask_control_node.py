import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String  
from flask import Flask, jsonify, request  
import threading  
import time  
from flask_cors import CORS 

# Initialize Flask app and enable CORS
app = Flask(__name__)
CORS(app)

stop_event = threading.Event()  # Event used to signal stopping of threads

# Define a ROS2 node class that integrates with Flask
class FlaskAppNode(Node):
    def __init__(self):
        super().__init__('flask_app')
        # Publisher to send robot parameters to the 'robot_parameters' topic
        self.publisher_ = self.create_publisher(String, 'robot_parameters', 1)
        # Publisher to send servo control commands to the 'robot_parameters' topic
        self.servo_publisher = self.create_publisher(String, 'robot_parameters', 1)

        # Subscription to listen for messages from the 'arduino_output' topic
        self.subscription_ = self.create_subscription(String, 'arduino_output', self.arduino_callback, 10)
        self.timer_ = None  # Placeholder for a timer (not used in this code)
        self.received_data = None  # Stores data received from Arduino
        self.arduino_connected = False  # Flag to check if Arduino is connected

    # Callback function triggered when a message is received on the 'arduino_output' topic
    def arduino_callback(self, msg):
        self.received_data = msg.data  # Store received data
        self.arduino_connected = True  # Set the connection flag to True

    # Function to publish a message with direction and speed to the 'robot_parameters' topic
    def publish_message(self, dir, speed):
        parity = str(int(dir) + int(speed))  # Simple parity check (sum of direction and speed)
        parameters_msg = String()
        parameters_msg.data = f"{dir},{speed},{parity};"  # Format message as 'direction,speed,parity;'
        self.publisher_.publish(parameters_msg)  # Publish the message

    # Function to publish servo angle to the 'robot_parameters' topic
    def publish_servo_angle(self, angle):
        servo_msg = String()

        # Format message for servo control -> -1, angle, -1 this is the format to let arduino detect that it's is for servo motor
        servo_msg.data = f"-1,{angle},-1;"  
        self.servo_publisher.publish(servo_msg)  # Publish the servo control message

# Flask route to serve a welcome message
@app.route('/')
def index():
    return jsonify({'message': 'Welcome to the Flask-control ROS2 App!'})

# Flask route to check if Arduino is connected
@app.route('/arduino_connection')
def arduino_connection():
    arduino_connected = flask_app_node.arduino_connected
    return jsonify({'arduino_connected': arduino_connected})

# Flask route to get the last received message from Arduino
@app.route('/arduino_message')
def arduino_response():
    received_data = flask_app_node.received_data
    return jsonify({'received_data': received_data})

# Flask route to start sending commands to the ROS2 node
@app.route('/receive', methods=['POST'])
def receive():
    global receiving
    data = request.get_json()
    dir = data.get('direction')  # Extract direction from the received JSON
    speed = data.get('speed')  # Extract speed from the received JSON

    receiving = True  # Set receiving flag to True
    # Start a new thread to send messages continuously
    sending_thread = threading.Thread(target=send_continuous_messages, args=(dir, speed))
    sending_thread.start()

    response = {'message': 'Command sent to ROS2 node!'}
    return jsonify(response)

# Flask route to stop sending commands to the ROS2 node
@app.route('/stop_receiving', methods=['POST'])
def stop_receiving():
    global receiving
    receiving = False  # Set receiving flag to False
    response = {'message': 'Command sent to ROS2 node!'}
    return jsonify(response)

# Flask route to control the servo by sending an angle
@app.route('/servo', methods=['POST'])
def control_servo():
    data = request.get_json()
    angle = data.get('angle')  # Extract angle from the received JSON
    flask_app_node.publish_servo_angle(angle)  # Send the angle to the ROS2 node
    return jsonify({'message': f'Servo angle {angle} sent to ROS2 node!'})

# Function to send messages continuously until stopped
def send_continuous_messages(direction, speed):
    global receiving
    while receiving:
        flask_app_node.publish_message(direction, speed)  # Publish direction and speed
        time.sleep(0.1)  # Wait for 100ms before sending the next message

# Main function to start the ROS2 node and Flask app
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    global flask_app_node
    flask_app_node = FlaskAppNode()  

    # Start a separate thread for the ROS2 node's spin loop
    ros_thread = threading.Thread(target=lambda: rclpy.spin(flask_app_node))
    ros_thread.start()

    app.run(host='0.0.0.0', port=3001)  # Start the Flask app, port number can change but if it change, it have to change all of node that connecting this flask node

    ros_thread.join()  

    rclpy.shutdown() 

if __name__ == '__main__':
    main()  
