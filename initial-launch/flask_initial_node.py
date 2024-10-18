from flask import Flask, jsonify
import subprocess
import os

app = Flask(__name__)

# Define the paths to the bash scripts for each service
SCRIPT_PATHS = {

    'launch_camera_service': os.path.expanduser('/home/cms-eam/Desktop/cms-inspection-robot-2024/camera-node/launch.sh'),
    'kill_camera_service': os.path.expanduser('/home/cms-eam/Desktop/cms-inspection-robot-2024/camera-node/kill_launch.sh'),
    'launch_sensors_service': os.path.expanduser('/home/cms-eam/Desktop/cms-inspection-robot-2024/sensors-node/launch.sh'),
    'kill_sensors_service': os.path.expanduser('/home/cms-eam/Desktop/cms-inspection-robot-2024/sensors-node/kill_launch.sh')
}

def run_bash_script(script_name):
    try:
        # Ensure the script name is valid
        script_path = SCRIPT_PATHS.get(script_name)
        print(script_path)
        if not script_path:
            raise ValueError(f"Invalid script name: {script_name}")

        # Check if the script exists at the specified path
        if not os.path.exists(script_path):
            raise FileNotFoundError(f"The script {script_path} does not exist.")

        # Directly execute the script using bash
        
        # subprocess.run(["bash", f"{script_path}"], shell= True)
        subprocess.run(f'{script_path}')
      
        return f"Finished for running {script_path}"
       
        

        status_code = 200 if result.returncode == 0 else 400
        return jsonify(response), status_code

    except Exception as e:
        # If there is an error, return it
        print(f"Exception: {str(e)}")  # Debugging line
        return jsonify({'error': str(e)}), 500


@app.route('/', methods=['GET'])
def home():
    return "Welcome to the CMS Inspection Robot Service!"



@app.route('/launch-camera-service', methods=['GET'])
def launch_camera_service():
    return run_bash_script('launch_camera_service')

@app.route('/kill-camera-service', methods=['GET'])
def kill_camera_service():
    return run_bash_script('kill_camera_service')

@app.route('/launch-sensors-service', methods=['GET'])
def launch_sensors_service():
    return run_bash_script('launch_sensors_service')

@app.route('/kill-sensors-service', methods=['GET'])
def kill_sensors_service():
    return run_bash_script('kill_sensors_service')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=10000)

