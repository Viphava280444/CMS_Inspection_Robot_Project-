import React from "react";
import { Navbar, Container } from "react-bootstrap";
import "bootstrap/dist/css/bootstrap.min.css";
import image from "../image/images.png";

const NavBar = () => {
  return (
    <Navbar className="navbar-custom" variant="dark">
      <Container fluid>
        <Navbar.Brand href="#home" className="d-flex align-items-center">
          <img
            alt=""
            src={image}
            width="50"
            height="50"
            className="d-inline-block align-top"
          />{" "}
          <span className="ms-3">CMS Inspection Robot</span>
        </Navbar.Brand>
      </Container>
    </Navbar>
  );
};

export default NavBar;
