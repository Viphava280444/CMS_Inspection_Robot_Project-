from flask import Flask, request, jsonify
from influxdb import InfluxDBClient
from datetime import datetime
import logging

app = Flask(__name__)

# Configuration for InfluxDB
host = "dbod-cms-inspection-robot.cern.ch"
port = 8091
username = "admin"
password = "changeme"
database = "sensors"

# Create the client
client = InfluxDBClient(
    host=host,
    port=port,
    username=username,
    password=password,
    database=database,
    ssl=True,
    verify_ssl=False
)

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

@app.route("/data", methods=["POST"])
def receive_data():
    try:
        data = request.json
        logger.debug(f"Received data: {data}")
        
        # Create a JSON body for InfluxDB
        json_body = [
            {
                "measurement": "sensor_data",
                "tags": {
                    "sensor": "arduino"
                },
                "fields": {
                    "temperature": float(data["temperature"]),
                    "gas": float(data["gas"]),
                    "sound": float(data["sound"]),
                    "magnetic": int(data["magnetic"])
                },
                "time": datetime.utcnow().isoformat()
            }
        ]
        
        logger.debug(f"JSON body created: {json_body}")
        client.write_points(json_body)
        logger.debug("Data written to InfluxDB successfully")
        return jsonify({"message": "Data received"}), 200
    except KeyError as e:
        logger.error(f"Invalid data format: {e}")
        return jsonify({"message": "Invalid data format", "error": str(e)}), 400
    except Exception as e:
        logger.error(f"Error writing to InfluxDB: {e}")
        return jsonify({"message": "Failed to write data", "error": str(e)}), 500

if __name__ == "__main__":
    app.run(debug=True, port=5001)
