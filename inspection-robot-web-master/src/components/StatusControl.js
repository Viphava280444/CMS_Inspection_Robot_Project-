import React, { useState, useEffect } from "react";
import { Card, Button, Form, Row, Col } from "react-bootstrap";
import config from "../config";

const StatusControl = ({ mode, setMode, onError }) => {
  const [servoAngle, setServoAngle] = useState(25);
  const [jetsonStatus, setJetsonStatus] = useState("Unknown");
  const [points, setPoints] = useState({
    top_left_x: 450,
    top_left_y: 550,
    top_right_x: 1478,
    top_right_y: 550,
    bottom_left_x: 450,
    bottom_left_y: 1080,
    bottom_right_x: 1478,
    bottom_right_y: 1080,
  });

  const sendServoAngle = async () => {
    try {
      const payload = { angle: servoAngle };
      await fetch(`${config.baseURL}:3001/servo`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const pingJetsonNano = async () => {
    try {
      const response = await fetch(`${config.baseURL}:3001`);
      if (response.ok) {
        setJetsonStatus("Active");
      } else {
        setJetsonStatus("Inactive");
      }
    } catch (error) {
      console.error(error);
      setJetsonStatus("Inactive");
    }
  };

  const switchMode = async (newMode) => {
    try {
      setMode(newMode);
      const payload = { mode: newMode };
      const response = await fetch(`${config.baseURL}:5004/switch_mode`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        console.error("Failed to switch mode:", response.statusText);
        onError(true);
        setMode((prevMode) => (prevMode === "normal" ? "lane" : "normal"));
      }
    } catch (error) {
      console.error("Error during mode switch:", error);
      onError(true);
      setMode((prevMode) => (prevMode === "normal" ? "lane" : "normal"));
    }
  };

  const updatePoints = async () => {
    try {
      await fetch(`${config.baseURL}:5004/points`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(points),
      });
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const startLaneDetection = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:5004/start_lane_detection`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
        }
      );
      if (!response.ok) {
        throw new Error("Failed to start lane detection");
      }
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const stopLaneDetection = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:5004/stop_lane_detection`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
        }
      );
      if (!response.ok) {
        throw new Error("Failed to stop lane detection");
      }
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const openCamera = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:10000/launch-camera-service`,
        {
          method: "GET",
        }
      );
    } catch (error) {
    }
  };

  const closeCamera = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:10000/kill-camera-service`,
        {
          method: "GET",
        }
      );

    } catch (error) {

    }
  };

  const openSensors = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:10000/launch-sensors-service`,
        {
          method: "GET",
        }
      );

    } catch (error) {
  
    }
  };

  const closeSensors = async () => {
    try {
      const response = await fetch(
        `${config.baseURL}:10000/kill-sensors-service`,
        {
          method: "GET",
        }
      );

    } catch (error) {

    }
  };



  useEffect(() => {
    sendServoAngle();
  }, [servoAngle]);

  useEffect(() => {
    const interval = setInterval(() => {
      pingJetsonNano();
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    updatePoints();
  }, [points]);



  const openLink = () => {
    window.open(
      "http://cms-eam-jetbot.cern.ch:3000/d/edtwxqoi5qfwgf/sensors?orgId=1&refresh=500ms&from=now-5m&to=now",
      "_blank"
    );
  };

  return (
    <Card className="control-card">
      <Card.Body>
        <div className="status-display">
          <span className="status-label">Status: </span>
          <span className={`status-value ${jetsonStatus.toLowerCase()}`}>
            {jetsonStatus}
          </span>
        </div>
        <div className="center-button">
          <Button variant="primary" onClick={openLink}>
            Open Sensor Dashboard
          </Button>
        </div>

        <div className="mode-switch mt-4 d-flex justify-content-center">
          <Button
            variant={mode === "normal" ? "success" : "outline-secondary"}
            onClick={() => switchMode("normal")}
            className="mr-2"
          >
            Normal Mode
          </Button>
          <Button
            variant={mode === "lane" ? "success" : "outline-secondary"}
            onClick={() => switchMode("lane")}
          >
            Lane Detection Mode
          </Button>
        </div>

        {mode === "normal" && (
          <div className="camera-sensors-control mt-4 d-flex justify-content-center">
            <Button
              variant="success"
              onClick={openCamera}
              className="mr-2"
            >
              Open Camera
            </Button>
            <Button
              variant="danger"
              onClick={closeCamera}
              className="mr-2"
            >
              Close Camera
            </Button>
            <Button
              variant="success"
              onClick={openSensors}
              className="mr-2"
            >
              Open Sensors
            </Button>
            <Button
              variant="danger"
              onClick={closeSensors}
              className="mr-2"
            >
              Close Sensors
            </Button>
          </div>
        )}

        {mode === "lane" && (
          <>
            <div className="lane-control mt-4">
              <Row>
                <Col>
                  <Form.Group>
                    <Form.Label>Top Left X: {points.top_left_x}</Form.Label>
                    <Form.Range
                      min="0"
                      max="1920"
                      value={points.top_left_x}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          top_left_x: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                  <Form.Group>
                    <Form.Label>Top Left Y: {points.top_left_y}</Form.Label>
                    <Form.Range
                      min="0"
                      max="1080"
                      value={points.top_left_y}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          top_left_y: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                </Col>
                <Col>
                  <Form.Group>
                    <Form.Label>Top Right X: {points.top_right_x}</Form.Label>
                    <Form.Range
                      min="0"
                      max="1920"
                      value={points.top_right_x}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          top_right_x: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                  <Form.Group>
                    <Form.Label>Top Right Y: {points.top_right_y}</Form.Label>
                    <Form.Range
                      min="0"
                      max="1080"
                      value={points.top_right_y}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          top_right_y: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                </Col>
              </Row>
              <Row>
                <Col>
                  <Form.Group>
                    <Form.Label>
                      Bottom Left X: {points.bottom_left_x}
                    </Form.Label>
                    <Form.Range
                      min="0"
                      max="1920"
                      value={points.bottom_left_x}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          bottom_left_x: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                  <Form.Group>
                    <Form.Label>
                      Bottom Left Y: {points.bottom_left_y}
                    </Form.Label>
                    <Form.Range
                      min="0"
                      max="1080"
                      value={points.bottom_left_y}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          bottom_left_y: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                </Col>
                <Col>
                  <Form.Group>
                    <Form.Label>
                      Bottom Right X: {points.bottom_right_x}
                    </Form.Label>
                    <Form.Range
                      min="0"
                      max="1920"
                      value={points.bottom_right_x}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          bottom_right_x: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                  <Form.Group>
                    <Form.Label>
                      Bottom Right Y: {points.bottom_right_y}
                    </Form.Label>
                    <Form.Range
                      min="0"
                      max="1080"
                      value={points.bottom_right_y}
                      onChange={(e) =>
                        setPoints({
                          ...points,
                          bottom_right_y: parseInt(e.target.value),
                        })
                      }
                    />
                  </Form.Group>
                </Col>
              </Row>
            </div>
            <div className="d-flex justify-content-center mt-3">
              <Button variant="primary" onClick={startLaneDetection}>
                Start Lane Detection
              </Button>
              <Button
                variant="danger"
                onClick={stopLaneDetection}
                className="ml-2"
              >
                Stop Lane Detection
              </Button>
            </div>
          </>
        )}

        <div className="servo-control mt-4">
          <Form.Group>
            <Form.Label>Servo Angle: {servoAngle}°</Form.Label>
            <Form.Range
              min="0"
              max="180"
              value={servoAngle}
              onChange={(e) => setServoAngle(parseInt(e.target.value))}
            />
          </Form.Group>
        </div>
      </Card.Body>
    </Card>
  );
};

export default StatusControl;
