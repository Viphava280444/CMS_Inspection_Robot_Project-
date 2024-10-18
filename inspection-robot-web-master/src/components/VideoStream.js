import React from "react";
import { Card } from "react-bootstrap";

const VideoStream = ({ title, src }) => {
  return (
    <Card className="stream-card">
      <Card.Body className="p-0">
        <Card.Title className="text-center py-2">{title}</Card.Title>
        <img
          src={src}
          alt={title}
          className="w-100"
          style={{ maxHeight: "400px", objectFit: "cover" }}
        />
      </Card.Body>
    </Card>
  );
};

export default VideoStream;
