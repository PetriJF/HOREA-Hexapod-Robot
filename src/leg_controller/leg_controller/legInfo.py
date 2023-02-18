#!/usr/bin/env python3
from dataclasses import dataclass
import numpy as np

BASE_WIDTH = 65.0 # mm

@dataclass
class Point:
    x: float
    y: float    
    z: float

@dataclass
class legReferencing:
    """A class used to organize the 6 legs and their respective 18 servo motors in terms of side and motor connection on the I2C board. 
       NOTE: side = false represents left, side = true represents right"""
    # Represents the side of the legs
    side: bool
    
    # Origin representation
    origin_x: float
    origin_y: float

    # Represents the port on the board for each motor of a leg
    hip_port: int
    shoulder_port: int
    knee_port: int

    # Simple description of the leg
    index: int
    description: str


# Leg initializations
LB = legReferencing(False, BASE_WIDTH * round(np.sin(7.0*np.pi/6.0),2), BASE_WIDTH * round(np.cos(7*np.pi/6.0),2),0, 1, 2, 4, "Leg: <Left Back>")
LM = legReferencing(False, BASE_WIDTH * round(np.sin(3.0*np.pi/2.0),2), BASE_WIDTH * round(np.cos(3.0*np.pi/2.0),2), 4, 5, 6, 5, "Leg: <Left Middle>")
LF = legReferencing(False, BASE_WIDTH * round(np.sin(11.0*np.pi/6.0),2), BASE_WIDTH * round(np.cos(11.0*np.pi/6.0),2), 8, 9, 10, 6, "Leg: <Left Front>")
RB = legReferencing(True, BASE_WIDTH * round(np.sin(5.0*np.pi/6.0),2), BASE_WIDTH * round(np.cos(5.0*np.pi/6.0),2), 8, 9, 10, 3, "Leg: <Right Back>")
RM = legReferencing(True, BASE_WIDTH * round(np.sin(np.pi/2.0),2), BASE_WIDTH * round(np.cos(np.pi/2.0),2), 4, 5, 6, 2, "Leg: <Right Middle>")
RF = legReferencing(True, BASE_WIDTH * round(np.sin(np.pi/6.0),2), BASE_WIDTH * round(np.cos(np.pi/6.0),2), 0, 1, 2, 1, "Leg: <Right Front>")