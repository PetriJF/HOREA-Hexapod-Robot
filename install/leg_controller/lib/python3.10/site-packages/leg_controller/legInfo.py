#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class legReferencing:
    """A class used to organize the 6 legs and their respective 18 servo motors in terms of side and motor connection on the I2C board. 
       NOTE: side = false represents left, side = true represents right"""
    side: bool
    
    hip: int
    shoulder: int
    knee: int

    description: str

LB = legReferencing(False, 0, 1, 2, "Leg: <Left Back>")
LM = legReferencing(False, 3, 4, 5, "Leg: <Left Middle>")
LF = legReferencing(False, 6, 7, 8, "Leg: <Left Front>")
RB = legReferencing(True, 0, 1, 2, "Leg: <Right Back>")
RM = legReferencing(True, 3, 4, 5, "Leg: <Right Middle>")
RF = legReferencing(True, 6, 7, 8, "Leg: <Right Front>")
