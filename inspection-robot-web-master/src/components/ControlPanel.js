import React from "react";
import { Card, Button } from "react-bootstrap";

const ControlPanel = () => {
  return (
    <Card className="control-card">
      <Card.Body className="control-panel">
        <Card.Title>Robot Control Panel</Card.Title>
        <Button variant="outline-light" className="mb-2">
          Move Forward
        </Button>
        <Button variant="outline-light" className="mb-2">
          Move Backward
        </Button>
        <Button variant="outline-light" className="mb-2">
          Turn Left
        </Button>
        <Button variant="outline-light" className="mb-2">
          Turn Right
        </Button>
        <Button variant="outline-light" className="mb-2">
          Stop
        </Button>
      </Card.Body>
    </Card>
  );
};

export default ControlPanel;
