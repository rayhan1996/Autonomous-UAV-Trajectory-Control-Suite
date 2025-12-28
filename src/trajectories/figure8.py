"""
Figure-8 trajectory generator.

This module provides a pure mathematical reference trajectory
for autonomous UAV flight planning.

No PX4 / MAVSDK code here.
"""

import math
from typing import Tuple


class Figure8Trajectory:
    def __init__(
        self,
        radius: float = 3.0,
        center_x: float = 0.0,
        center_y: float = 0.0,
        omega: float = 0.3,
    ):
        self.R = radius
        self.cx = center_x
        self.cy = center_y
        self.w = omega

    def position_xy(self, t: float) -> Tuple[float, float]:
        """
        Compute XY reference at time t.

        x = R * sin(w t)
        y = 0.5 R * sin(2 w t)
        """
        x = self.cx + self.R * math.sin(self.w * t)
        y = self.cy + 0.5 * self.R * math.sin(2 * self.w * t)
        return x, y

    def duration(self) -> float:
        """
        Nominal duration for one full figure-8.
        """
        return 2 * math.pi / self.w

    def yaw_deg(self, t: float) -> float:
        """
        Heading (yaw) aligned with trajectory velocity.
        Returned in degrees (NED frame).
        """
        vx = self.R * self.w * math.cos(self.w * t)
        vy = self.R * self.w * math.cos(2 * self.w * t)
    
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return 0.0
    
        yaw_rad = math.atan2(vy, vx)
        return math.degrees(yaw_rad)

