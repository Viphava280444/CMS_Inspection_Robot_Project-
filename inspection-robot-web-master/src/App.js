import React, { useState, useEffect } from "react";
import NavBar from "./components/NavBar";
import HSVControls from "./components/HSVControls";
import VideoStream from "./components/VideoStream";
import StatusControl from "./components/StatusControl";
import Controller from "./components/Controller";
import "bootstrap/dist/css/bootstrap.min.css";
import { Container, Row, Col } from "react-bootstrap";
import "./index.css";
import config from "./config";

function App() {
  const [hsv, setHsv] = useState({
    h_min: 0,
    h_max: 179,
    s_min: 0,
    s_max: 255,
    v_min: 0,
    v_max: 255,
  });

  const [mode, setMode] = useState("normal");

  useEffect(() => {
    const fetchData = async () => {
      try {
        // Fetch initial HSV values
        const response = await fetch(`${config.baseURL}:5004/hsv`);
        const data = await response.json();
        setHsv(data);

        // Set up mode switch handling
        const modeResponse = await fetch(`${config.baseURL}:5004/switch_mode`);
        const modeData = await modeResponse.json();
        setMode(modeData.mode);
      } catch (error) {
        // Silently handle errors (do nothing)
      }
    };

    fetchData();
  }, []);

  const handleChange = async (e) => {
    const { name, value } = e.target;
    setHsv((prevHsv) => ({
      ...prevHsv,
      [name]: parseInt(value),
    }));

    try {
      // Send updated HSV values to the server
      await fetch(`${config.baseURL}:5004/hsv`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ [name]: parseInt(value) }),
      });
    } catch (error) {
      // Silently handle errors (do nothing)
    }
  };

  const handleError = (hasError) => {
    if (hasError) {
      // Silently handle the error (do nothing)
    }
  };

  return (
    <div className="App">
      <NavBar />
      <Container fluid className="pt-3">
        <Row className="mb-3">
          <Col md={4}>
            <VideoStream
              key={`original-${mode}`}
              title="Original View"
              src={`${config.baseURL}:5004/video_${
                mode === "normal" ? "original" : "lane_original"
              }`}
            />
          </Col>
          <Col md={4}>
            <VideoStream
              key={`mask-${mode}`}
              title="Mask View"
              src={`${config.baseURL}:5004/video_${
                mode === "normal" ? "mask" : "lane_mask"
              }`}
            />
          </Col>
          <Col md={4}>
            <VideoStream
              key={`combined-${mode}`}
              title="Combined View"
              src={`${config.baseURL}:5004/video_${
                mode === "normal" ? "combined" : "lane_combined"
              }`}
            />
          </Col>
        </Row>
        <Row>
          <Col md={4} className="mb-3">
            <HSVControls hsv={hsv} handleChange={handleChange} />
          </Col>
          <Col md={4} className="mb-3">
            <Controller onError={handleError} />
          </Col>
          <Col md={4} className="mb-3">
            <StatusControl
              mode={mode}
              setMode={setMode}
              onError={handleError}
            />
          </Col>
        </Row>
      </Container>
    </div>
  );
}

export default App;
