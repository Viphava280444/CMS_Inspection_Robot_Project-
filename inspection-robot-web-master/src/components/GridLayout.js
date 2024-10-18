import React from "react";
import { Container, Row, Col, Card, Button } from "react-bootstrap";
import "bootstrap/dist/css/bootstrap.min.css";

const GridLayout = () => {
  return (
    <Container fluid className="full-height-container pt-2">
      <Row className="full-height-row">
        <Col md={4} className="full-height-col mb-4">
          <Card className="stream-card">
            <Card.Body>
              <Card.Text>Video stream 1</Card.Text>
            </Card.Body>
          </Card>
        </Col>
        <Col md={4} className="full-height-col mb-4">
          <Card className="stream-card">
            <Card.Body>
              <Card.Text>Video stream 2</Card.Text>
            </Card.Body>
          </Card>
        </Col>
        <Col md={4} className="full-height-col mb-4">
          <Card className="stream-card">
            <Card.Body>
              <Card.Text>Video stream 3</Card.Text>
            </Card.Body>
          </Card>
        </Col>
      </Row>
      <Row className="full-height-row">
        <Col md={4} className="full-height-col mb-4">
          <Card className="control-card">
            <Card.Body>
              <Card.Title>HSV Adjustment</Card.Title>
              <Card.Text>HSV controls here</Card.Text>
            </Card.Body>
          </Card>
        </Col>
        <Col md={4} className="full-height-col mb-4">
          <Card className="control-card">
            <Card.Body className="control-panel">
              <Card.Title>Robot Control Panel</Card.Title>
              <Button variant="outline-light">Move Forward</Button>
              <Button variant="outline-light">Move Backward</Button>
              <Button variant="outline-light">Turn Left</Button>
              <Button variant="outline-light">Turn Right</Button>
              <Button variant="outline-light">Stop</Button>
            </Card.Body>
          </Card>
        </Col>
        <Col md={4} className="full-height-col mb-4">
          <Card className="control-card">
            <Card.Body>
              <Card.Title>Status and Control</Card.Title>
              <Card.Text>Status: Active/Inactive</Card.Text>
              <Button variant="success">Turn On</Button>
              <Button variant="danger">Turn Off</Button>
            </Card.Body>
          </Card>
        </Col>
      </Row>
    </Container>
  );
};

export default GridLayout;
