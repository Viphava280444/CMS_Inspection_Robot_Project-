import React, { useState } from "react";
import config from "../config";

const Controller = ({ onError }) => {
  const [currentSpeed, setCurrentSpeed] = useState("medium"); // Default speed level is medium
  const [currentSpeedData, setCurrentSpeedData] = useState(150); // Default speed level is medium
  const [rememberedSteps, setRememberedSteps] = useState([]);
  const [recordedSteps, setRecordedSteps] = useState([]);

  const onSpeedChange = (speed) => {
    let speedValue;

    setCurrentSpeed(speed);

    switch (speed) {
      case "low":
        speedValue = 100;
        break;
      case "medium":
        speedValue = 150;
        break;
      case "high":
        speedValue = 250;
        break;
      default:
        // Default to medium speed if an invalid speed level is provided
        speedValue = 150;
    }

    setCurrentSpeedData(speedValue);
  };

  const startSendingData = async (direction) => {
    try {
      const payload = { speed: currentSpeedData, direction };
      console.log(JSON.stringify(payload)); // Log the JSON payload for verification

      // Use HTTPS URL
      const response = await fetch(`${config.baseURL}:3001/receive`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
      });
      console.log(response); // Log the response for testing

      // Record the step
      setRecordedSteps((prevSteps) => [...prevSteps, direction]);
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const stopSendingData = async () => {
    try {
      const payload = { speed: currentSpeedData, direction: 0 }; // Replace currentDir with 0
      console.log(JSON.stringify(payload)); // Log the JSON payload for verification

      // Use HTTPS URL
      const response = await fetch(`${config.baseURL}:3001/stop_receiving`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      console.log(response);

      // Record the step
      setRecordedSteps((prevSteps) => [...prevSteps, 0]); // Replace currentDir with 0
    } catch (error) {
      console.error(error);
      onError(true);
    }
  };

  const handleSetHome = () => {
    setRecordedSteps([]); // Clear recorded steps
    setRememberedSteps([...recordedSteps]); // Remember the current recorded steps as the new home position
  };

  const handleGoHome = async () => {
    // Use the remembered steps in reverse order to return to the starting point
    const reversedSteps = rememberedSteps.slice().reverse();
    for (const direction of reversedSteps) {
      await new Promise((resolve) => setTimeout(resolve, 1000)); // Add a delay between steps
      await startSendingData(direction);
    }
  };

  return (
    <div className="container">
      <div className="controller-container">
        <div className="grid-container">
          <button
            className="dpad-button up-left"
            data-icon="&#xf048;"
            name="dir"
            value="6"
            onMouseDown={() => startSendingData(6)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button up"
            data-icon="&#xf062;"
            name="dir"
            value="0"
            onMouseDown={() => startSendingData(0)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button up-right"
            data-icon="&#xf049;"
            name="dir"
            value="7"
            onMouseDown={() => startSendingData(7)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button left"
            data-icon="&#xf060;"
            name="dir"
            value="2"
            onMouseDown={() => startSendingData(2)}
            onMouseUp={stopSendingData}
          ></button>
          <div className="empty-square"></div>
          <button
            className="dpad-button right"
            data-icon="&#xf061;"
            name="dir"
            value="3"
            onMouseDown={() => startSendingData(3)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button down-left"
            data-icon="&#xf04b;"
            name="dir"
            value="8"
            onMouseDown={() => startSendingData(8)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button down"
            data-icon="&#xf063;"
            name="dir"
            value="1"
            onMouseDown={() => startSendingData(1)}
            onMouseUp={stopSendingData}
          ></button>
          <button
            className="dpad-button down-right"
            data-icon="&#xf04c;"
            name="dir"
            value="9"
            onMouseDown={() => startSendingData(9)}
            onMouseUp={stopSendingData}
          ></button>
        </div>
        <div className="rotation-container">
          <button
            className="rotate-button clockwise flip-horizontal"
            name="dir"
            value="4"
            onMouseDown={() => startSendingData(4)}
            onMouseUp={stopSendingData}
          >
            <div className="rotate-icon">&#10227;</div>
          </button>
          <button
            className="rotate-button counterclockwise"
            name="dir"
            value="5"
            onMouseDown={() => startSendingData(5)}
            onMouseUp={stopSendingData}
          >
            <div className="rotate-icon">&#10227;</div>
          </button>
        </div>
      </div>
      <div className="speed-buttons">
        <button
          onClick={() => onSpeedChange("low")}
          className={currentSpeed === "low" ? "current-speed" : ""}
        >
          Low
        </button>
        <button
          onClick={() => onSpeedChange("medium")}
          className={currentSpeed === "medium" ? "current-speed" : ""}
        >
          Medium
        </button>
        <button
          onClick={() => onSpeedChange("high")}
          className={currentSpeed === "high" ? "current-speed" : ""}
        >
          High
        </button>
      </div>
      <div className="home-buttons">
        <button onClick={handleSetHome}>Set Home</button>
        <button onClick={handleGoHome}>Go Home</button>
      </div>
    </div>
  );
};

export default Controller;
