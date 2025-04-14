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
class FrameData:
    """Frame data with detected faces"""
    frame_width: int
    frame_height: int
    faces: List[FaceData]
    timestamp: float

    def to_json(self) -> str:
        """Convert to JSON string for ROS messages"""
        data = {
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'timestamp': self.timestamp,
            'faces': [face.to_dict() for face in self.faces]
        }
        return json.dumps(data)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'FrameData':
        """Create FrameData from JSON string"""
        data = json.loads(json_str)
        return cls(
            frame_width=data['frame_width'],
            frame_height=data['frame_height'],
            timestamp=data.get('timestamp', 0.0),
            faces=[FaceData.from_dict(face) for face in data['faces']]
        )


@dataclass
class VelocityVector:
    """Velocity vector for tracking movement"""
    x: float
    y: float
    magnitude: float

    @classmethod
    def zero(cls) -> 'VelocityVector':
        """Create zero velocity vector"""
        return cls(0.0, 0.0, 0.0)


@dataclass
class MotorPosition:
    """Motor position data"""
    pan_angle: float  # in degrees
    tilt_angle: float  # in degrees
    
    @classmethod
    def default(cls) -> 'MotorPosition':
        """Default center position"""
        return cls(90.0, 180.0) 