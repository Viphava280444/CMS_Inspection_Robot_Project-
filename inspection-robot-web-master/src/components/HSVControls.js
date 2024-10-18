import React from "react";
import { Form, Row, Col, Card } from "react-bootstrap";

const predefinedColors = {
  white: { h_min: 0, h_max: 179, s_min: 0, s_max: 30, v_min: 200, v_max: 255 },
  green: {
    h_min: 35,
    h_max: 85,
    s_min: 100,
    s_max: 255,
    v_min: 50,
    v_max: 255,
  },
  yellow: {
    h_min: 20,
    h_max: 30,
    s_min: 100,
    s_max: 255,
    v_min: 100,
    v_max: 255,
  },
  red: { h_min: 0, h_max: 10, s_min: 100, s_max: 255, v_min: 100, v_max: 255 },
  black: { h_min: 0, h_max: 179, s_min: 0, s_max: 255, v_min: 0, v_max: 50 },
};

const HSVControls = ({ hsv, handleChange }) => {
  const handlePresetChange = (e) => {
    const selectedColor = e.target.value;
    if (predefinedColors[selectedColor]) {
      handleChange({
        target: { name: "h_min", value: predefinedColors[selectedColor].h_min },
      });
      handleChange({
        target: { name: "h_max", value: predefinedColors[selectedColor].h_max },
      });
      handleChange({
        target: { name: "s_min", value: predefinedColors[selectedColor].s_min },
      });
      handleChange({
        target: { name: "s_max", value: predefinedColors[selectedColor].s_max },
      });
      handleChange({
        target: { name: "v_min", value: predefinedColors[selectedColor].v_min },
      });
      handleChange({
        target: { name: "v_max", value: predefinedColors[selectedColor].v_max },
      });
    }
  };

  return (
    <Card className="control-card">
      <Card.Body>
        <Card.Title>HSV Controls</Card.Title>
        <Form>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              Presets
            </Form.Label>
            <Col sm="10">
              <Form.Select
                onChange={handlePresetChange}
                className="futuristic-dropdown"
              >
                <option value="">Select a preset</option>
                <option value="white">⚪ White</option>
                <option value="green">🟢 Green</option>
                <option value="yellow">🟡 Yellow</option>
                <option value="red">🔴 Red</option>
                <option value="black">⚫ Black</option>
              </Form.Select>
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              H_MAX
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="h_max"
                min="0"
                max="179"
                value={hsv.h_max}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.h_max} />
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              H_MIN
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="h_min"
                min="0"
                max="179"
                value={hsv.h_min}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.h_min} />
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              S_MAX
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="s_max"
                min="0"
                max="255"
                value={hsv.s_max}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.s_max} />
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              S_MIN
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="s_min"
                min="0"
                max="255"
                value={hsv.s_min}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.s_min} />
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              V_MAX
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="v_max"
                min="0"
                max="255"
                value={hsv.v_max}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.v_max} />
            </Col>
          </Form.Group>
          <Form.Group as={Row} className="mb-3">
            <Form.Label column sm="2">
              V_MIN
            </Form.Label>
            <Col sm="8">
              <Form.Control
                type="range"
                name="v_min"
                min="0"
                max="255"
                value={hsv.v_min}
                onChange={handleChange}
              />
            </Col>
            <Col sm="2">
              <Form.Control type="text" readOnly value={hsv.v_min} />
            </Col>
          </Form.Group>
        </Form>
      </Card.Body>
    </Card>
  );
};

export default HSVControls;
