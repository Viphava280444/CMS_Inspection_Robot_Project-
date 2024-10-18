import cv2
import numpy as np
from pyzbar.pyzbar import decode
import depthai as dai
import threading
import aiohttp
import json
from flask import Flask, Response, request, jsonify
from flask_cors import CORS
import time
import asyncio

# Global variables
latest_frame = None  # To store the latest frame from the camera
processing_qr_code = False  # To prevent multiple QR codes from being processed simultaneously
frame_lock = threading.Lock()  # To safely update global variables between threads
bounding_boxes = []  # To store the coordinates of detected QR codes
mode = "normal"  # To switch between different modes of operation
lane_detection_active = False  # To check if lane detection is currently active
perspective_matrix = None  # For perspective transformation in lane detection mode

# Default points for perspective transformation (adjusted for 1280x720 resolution)
points = {
    "top_left_x": 320,
    "top_left_y": 180,
    "top_right_x": 960,
    "top_right_y": 180,
    "bottom_left_x": 320,
    "bottom_left_y": 540,
    "bottom_right_x": 960,
    "bottom_right_y": 540
}

# QR code detection function
async def decode_barcodes(frame):
    global processing_qr_code, bounding_boxes
    if processing_qr_code:
        return
    
    current_bounding_boxes = []
    
    # Loop through detected barcodes in the frame
    for barcode in decode(frame):
        myData = barcode.data.decode("utf-8")
        try:
            data = json.loads(myData)
    
            # Extract speed, direction, and duration from the QR code data
            speed = data["speed"]
            direction = data["direction"]
            duration = data["duration"]

            # Mark as processing to prevent further detection
            processing_qr_code = True

            # Start a task to handle the sending logic
            asyncio.create_task(handle_qr_code_data(speed, direction, duration))
        except Exception as e:
            print(f"Failed to process QR code: {e}")
        
        # Get bounding box coordinates for drawing
        points = barcode.polygon
        if len(points) == 4:
            current_bounding_boxes.append(([(point.x, point.y) for point in points], myData))
    
    # Update the global bounding boxes
    with frame_lock:
        bounding_boxes = current_bounding_boxes

# Function to handle the QR code data by sending it to a Flask server
async def handle_qr_code_data(speed, direction, duration):
    global processing_qr_code
    try:
        async with aiohttp.ClientSession() as session:
            start_time = time.time()
            end_time = start_time + duration
            while time.time() < end_time:
                payload = {"speed": speed, "direction": direction}
                async with session.post("http://localhost:3001/receive", json=payload) as response:
                    await response.json()
                 
                await asyncio.sleep(0.1)  # Sleep for a short duration before sending the next message

            # Send stop command
            stop_payload = {"speed": 0, "direction": 0}
            async with session.post("http://localhost:3001/stop_receiving", json=stop_payload) as response:
                await response.json()
        
    except Exception as e:
        print(f"Failed to send data to Flask server: {e}")
    finally:
        # Allow new QR code detection
        processing_qr_code = False

# Class to handle OAK-D camera streaming
class OakDStream:
    def __init__(self):
        self.pipeline = dai.Pipeline()

        # Set up the color camera
        self.cam_rgb = self.pipeline.createColorCamera()
        self.cam_rgb.setPreviewSize(640, 480)  # Lower resolution for faster transmission
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        # Create output for the camera
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.video.link(self.xout_rgb.input)

        self.frame = None  # To store the current frame
        self.running = True  # Flag to keep the camera running
        self.lock = threading.Lock()  # Lock for safe access to the frame

    def start(self):
        try:
            self.device = dai.Device(self.pipeline)  # Initialize the OAK-D device
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)  # Queue for RGB frames
            self.thread = threading.Thread(target=self.update, args=())  # Thread for updating frames
            self.thread.daemon = True  # Ensure the thread closes when the main program exits
            self.thread.start()
        except Exception as e:
            print(f"Failed to start device: {e}")
            self.running = False
        return self

    def update(self):
        while self.running:
            if self.device is None:
                continue
            try:
                in_rgb = self.q_rgb.get()  # Blocking call, will wait until new data has arrived
                frame = in_rgb.getCvFrame()  # Convert the frame to OpenCV format
                with self.lock:
                    self.frame = frame  # Update the current frame
            except Exception as e:
                print(f"An error occurred while fetching frame: {e}")

    def get_frame(self):
        with self.lock:
            return self.frame  # Return the current frame

    def stop(self):
        self.running = False  # Stop the camera stream
        if self.device:
            self.device.close()  # Close the device
        if self.thread.is_alive():
            self.thread.join()  # Wait for the thread to finish

# Initialize the OAK-D stream
stream = OakDStream().start()

# Default HSV values to detect white color
hsv_values = {
    "h_min": 0,
    "h_max": 179,
    "s_min": 0,
    "s_max": 30,
    "v_min": 200,
    "v_max": 255
}

app = Flask(__name__)  # Initialize the Flask app
CORS(app)  # Enable Cross-Origin Resource Sharing

# Function to start the asyncio event loop in a separate thread
def start_event_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

# Create and start the event loop thread
loop = asyncio.new_event_loop()
threading.Thread(target=start_event_loop, args=(loop,), daemon=True).start()

# Function to generate frames from the camera stream and process them
def generate_frames(frame_processor, run_qr_detection=False):
    frame_skip = 0
    while True:
        frame = stream.get_frame()
        if frame is not None:
            if run_qr_detection and frame_skip % 5 == 0:  # Process every 5th frame for QR code detection
                asyncio.run_coroutine_threadsafe(decode_barcodes(frame), loop)
            frame_skip += 1

            # Check if lane detection is active and mode is 'lane'
            if lane_detection_active and mode == "lane":
                frame = apply_lane_detection(frame)  # Apply lane detection

            with frame_lock:
                current_bounding_boxes = bounding_boxes.copy()  # Get the current bounding boxes
            frame = frame_processor(frame, current_bounding_boxes)  # Process the frame with the given processor
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])  # Reduce JPEG quality for faster transmission
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)  # Add a small delay to reduce CPU usage

# Route to stream the original video with QR code detection
@app.route('/video_original')
def video_feed1():
    def normal_frame_processor(frame, bounding_boxes):
        for box, data in bounding_boxes:
            if len(box) == 4:
                # Draw bounding boxes around detected QR codes
                cv2.line(frame, box[0], box[1], (0, 255, 0), 2)
                cv2.line(frame, box[1], box[2], (0, 255, 0), 2)
                cv2.line(frame, box[2], box[3], (0, 255, 0), 2)
                cv2.line(frame, box[3], box[0], (0, 255, 0), 2)
                # Add label to the detected QR code
                cv2.putText(frame, data, (box[0][0], box[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame
    return Response(generate_frames(normal_frame_processor, run_qr_detection=True), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to stream the video with a mask applied
@app.route('/video_mask')
def video_feed2():
    def mask_frame_processor(frame, bounding_boxes):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min'])
        upper = (hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max'])
        mask = cv2.inRange(hsv_frame, lower, upper)
        return mask
    return Response(generate_frames(mask_frame_processor), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to stream the video with both the mask and the original frame combined
@app.route('/video_combined')
def video_feed3():
    def combined_frame_processor(frame, bounding_boxes):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min'])
        upper = (hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max'])
        mask = cv2.inRange(hsv_frame, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        return result
    return Response(generate_frames(combined_frame_processor), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to stream the video in lane detection mode with original frame and lane markers
@app.route('/video_lane_original')
def video_feed4():
    if mode != "lane":
        return jsonify({"error": "Not in lane detection mode"}), 400

    def lane_original_frame_processor(frame, bounding_boxes):
        # Draw points and lines based on the four adjustable points
        pt1 = (points["top_left_x"], points["top_left_y"])
        pt2 = (points["top_right_x"], points["top_right_y"])
        pt3 = (points["bottom_left_x"], points["bottom_left_y"])
        pt4 = (points["bottom_right_x"], points["bottom_right_y"])

        cv2.circle(frame, pt1, 5, (255, 0, 0), -1)
        cv2.circle(frame, pt2, 5, (255, 0, 0), -1)
        cv2.circle(frame, pt3, 5, (255, 0, 0), -1)
        cv2.circle(frame, pt4, 5, (255, 0, 0), -1)

        cv2.line(frame, pt1, pt2, (255, 0, 0), 2)
        cv2.line(frame, pt2, pt4, (255, 0, 0), 2)
        cv2.line(frame, pt4, pt3, (255, 0, 0), 2)
        cv2.line(frame, pt3, pt1, (255, 0, 0), 2)
       
        return frame
    return Response(generate_frames(lane_original_frame_processor), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to stream the video in lane detection mode with a mask applied
@app.route('/video_lane_mask')
def video_feed5():
    if mode != "lane":
        return jsonify({"error": "Not in lane detection mode"}), 400

    def lane_mask_frame_processor(frame, bounding_boxes):
        global perspective_matrix
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min'])
        upper = (hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max'])
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Apply the perspective transform (compute only once)
        if perspective_matrix is None:
            src_pts = np.array([
                [points["top_left_x"], points["top_left_y"]],
                [points["top_right_x"], points["top_right_y"]],
                [points["bottom_left_x"], points["bottom_left_y"]],
                [points["bottom_right_x"], points["bottom_right_y"]]
            ], dtype=np.float32)

            dst_pts = np.array([
                [0, 0],
                [1920, 0],
                [0, 1080],
                [1920, 1080]
            ], dtype=np.float32)

            perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        mask_transformed = cv2.warpPerspective(mask, perspective_matrix, (1920, 1080))

        return cv2.cvtColor(mask_transformed, cv2.COLOR_GRAY2BGR)  # Convert mask to BGR format for display
    return Response(generate_frames(lane_mask_frame_processor), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to stream the video in lane detection mode with both the mask and the original frame combined
@app.route('/video_lane_combined')
def video_feed6():
    if mode != "lane":
        return jsonify({"error": "Not in lane detection mode"}), 400

    def lane_combined_frame_processor(frame, bounding_boxes):
        global perspective_matrix
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min'])
        upper = (hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max'])
        mask = cv2.inRange(hsv_frame, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Apply the perspective transform (compute only once)
        if perspective_matrix is None:
            src_pts = np.array([
                [points["top_left_x"], points["top_left_y"]],
                [points["top_right_x"], points["top_right_y"]],
                [points["bottom_left_x"], points["bottom_left_y"]],
                [points["bottom_right_x"], points["bottom_right_y"]]
            ], dtype=np.float32)

            dst_pts = np.array([
                [0, 0],
                [1920, 0],
                [0, 1080],
                [1920, 1080]
            ], dtype=np.float32)

            perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        result_transformed = cv2.warpPerspective(result, perspective_matrix, (1920, 1080))

        return result_transformed
    return Response(generate_frames(lane_combined_frame_processor), mimetype='multipart/x-mixed-replace; boundary=frame')

# Function to apply lane detection algorithm and control the robot
def apply_lane_detection(frame):
    global perspective_matrix
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = (hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min'])
    upper = (hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max'])
    mask = cv2.inRange(hsv_frame, lower, upper)

    # Apply perspective transform to get bird's eye view
    if perspective_matrix is None:
        src_pts = np.array([
            [points["top_left_x"], points["top_left_y"]],
            [points["top_right_x"], points["top_right_y"]],
            [points["bottom_left_x"], points["bottom_left_y"]],
            [points["bottom_right_x"], points["bottom_right_y"]]
        ], dtype=np.float32)

        dst_pts = np.array([
            [0, 0],
            [1920, 0],
            [0, 1080],
            [1920, 1080]
        ], dtype=np.float32)

        perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    
    warped = cv2.warpPerspective(mask, perspective_matrix, (1920, 1080))

    # Use the new lane detection algorithm
    processed_frame, direction = process_frame(mask)

    # Determine speed based on direction
    if direction == "Go Straight":
        speed = 100  # Speed when going straight
        direction_value = 0  # Assuming 0 is for straight
    elif direction == "Turn Left":
        speed = 90  # Speed when turning left
        direction_value = 4  # Assuming 4 is for left
    elif direction == "Turn Right":
        speed = 90  # Speed when turning right
        direction_value = 5  # Assuming 5 is for right
    else:  # Stop
        speed = 0
        direction_value = 0  # Stop the robot

    # Send command to control the robot
    asyncio.run_coroutine_threadsafe(send_control_command(direction_value, speed), loop)

    return processed_frame

# Function to send control commands to the robot
async def send_control_command(direction, speed, delay=0.1):
    try:
        async with aiohttp.ClientSession() as session:
            # Send the initial direction command with speed
            payload = {"direction": direction, "speed": speed}
            async with session.post("http://localhost:3001/receive", json=payload) as response:
                await response.json()

            # Introduce a small delay before sending the stop command
            await asyncio.sleep(delay)

            # Send the stop command (direction 0, speed 0)
            stop_payload = {"direction": 0, "speed": 0}
            async with session.post("http://localhost:3001/stop_receiving", json=stop_payload) as response:
                await response.json()

    except Exception as e:
        print(f"Failed to send control command: {e}")

# Function to process the frame for lane detection
def process_frame(mask_frame):
    # Ensure mask frame is binary (0 for black and 255 for white)
    _, binary_mask = cv2.threshold(mask_frame, 127, 255, cv2.THRESH_BINARY)

    # Define the sensor points (5 points across the bottom part of the image)
    height, width = binary_mask.shape
    sensor_y = int(height * 0.75)  # Sensor line at 3/4th of the frame height

    sensors = {
        'left': (int(width * 0.2), sensor_y),
        'center_left': (int(width * 0.4), sensor_y),
        'center': (int(width * 0.5), sensor_y),
        'center_right': (int(width * 0.6), sensor_y),
        'right': (int(width * 0.8), sensor_y),
    }

    sensor_values = {key: binary_mask[point[1], point[0]] for key, point in sensors.items()}

    # Decide direction based on sensor values
    if sensor_values['left'] == 255 or sensor_values['center_left'] == 255:
        direction = "Turn Left"
    elif sensor_values['right'] == 255 or sensor_values['center_right'] == 255:
        direction = "Turn Right"
    elif sensor_values['center'] == 255:
        direction = "Go Straight"
    else:
        direction = "Stop"  # Stop if no lane is detected

    # Visualize the sensors on the frame
    for key, point in sensors.items():
        color = (0, 255, 0) if sensor_values[key] == 255 else (0, 0, 255)
        cv2.circle(mask_frame, point, 10, color, -1)

    # Display the direction on the frame
    cv2.putText(mask_frame, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    return mask_frame, direction

# API to get or update HSV values
@app.route('/hsv', methods=['GET', 'POST'])
def hsv():
    global hsv_values
    
    if request.method == 'POST':
        data = request.json
        hsv_values.update(data)
        return jsonify(hsv_values)
    else:
        return jsonify(hsv_values)

# API to switch between different modes (normal or lane detection)
@app.route('/switch_mode', methods=['GET', 'POST'])
def switch_mode():
    global mode, lane_detection_active, perspective_matrix
    if request.method == 'POST':
        data = request.json
        if 'mode' in data:
            if data['mode'] in ['normal', 'lane']:
                mode = data['mode']
                lane_detection_active = False  # Reset lane detection activity
                perspective_matrix = None  # Reset perspective matrix
                return jsonify({"status": "Mode switched", "mode": mode}), 200
            else:
                return jsonify({"error": "Invalid mode"}), 400
        else:
            return jsonify({"error": "Mode not specified"}), 400
    else:
        return jsonify({'mode': mode})

# API to start lane detection
@app.route('/start_lane_detection', methods=['POST'])
def start_lane_detection():
    global lane_detection_active
    if mode != "lane":
        return jsonify({"error": "Not in lane detection mode"}), 400

    lane_detection_active = True
    return jsonify({"status": "Lane detection started"}), 200

# API to stop lane detection
@app.route('/stop_lane_detection', methods=['POST'])
def stop_lane_detection():
    global lane_detection_active
    lane_detection_active = False
    return jsonify({"status": "Lane detection stopped"}), 200

# API to get or update the points for perspective transformation
@app.route('/points', methods=['GET', 'POST'])
def points_handler():
    global points, perspective_matrix

    if request.method == 'POST':
        data = request.json
        points.update(data)
        perspective_matrix = None  # Recompute perspective matrix
        return jsonify(points)
    else:
        return jsonify(points)

# Root route
@app.route('/')
def index():
    return "OAK-D Stream Server"

# Main function to start the Flask app
if __name__ == "__main__":
    try:
        app.run(host='0.0.0.0', port=5004, threaded=True)
    except KeyboardInterrupt:
        stream.stop()
        loop.stop()
