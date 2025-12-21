"""
Circular trajectory generator.

Pure mathematical reference trajectory for autonomous UAV flight planning.
No PX4 / MAVSDK code here.
"""

import math
from typing import Tuple


class CircleTrajectory:
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

        x(t) = cx + R * cos(w t)
        y(t) = cy + R * sin(w t)
        """
        x = self.cx + self.R * math.cos(self.w * t)
        y = self.cy + self.R * math.sin(self.w * t)
        return x, y

    def duration(self) -> float:
        """
        Nominal duration for one full circle.
        """
        return 2 * math.pi / self.w
