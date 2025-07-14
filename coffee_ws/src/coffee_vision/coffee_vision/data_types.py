#!/usr/bin/env python3

import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Tuple, Optional


@dataclass
class FaceData:
    """Standard representation of face detection data"""
    x1: int
    y1: int
    x2: int
    y2: int
    center_x: int
    center_y: int
    confidence: float

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'FaceData':
        """Create FaceData from dictionary representation"""
        return cls(
            x1=data['x1'],
            y1=data['y1'],
            x2=data['x2'],
            y2=data['y2'],
            center_x=data['center_x'],
            center_y=data['center_y'],
            confidence=data['confidence']
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation"""
        return asdict(self)

    @property
    def width(self) -> int:
        """Get face width"""
        return self.x2 - self.x1
    
    @property
    def height(self) -> int:
        """Get face height"""
        return self.y2 - self.y1
    
    @property
    def area(self) -> int:
        """Get face area"""
        return self.width * self.height


@dataclass
class VelocityVector:
    """Representation of a 2D velocity vector"""
    x: float
    y: float
    magnitude: float 