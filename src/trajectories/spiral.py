"""
Spiral (Helix) trajectory generator.

Pure mathematical reference trajectory for autonomous UAV flight planning.
Independent of PX4, MAVSDK, or any flight control logic.

The trajectory defines a horizontal circular motion with a
monotonic change in altitude over time (3D path).
"""

import math
from typing import Tuple


class SpiralTrajectory:
    def __init__(
        self,
        radius: float = 3.0,
        center_x: float = 0.0,
        center_y: float = 0.0,
        start_z: float = 0.0,
        end_z: float = -5.0,
        omega: float = 0.3,
    ):
        """
        Parameters:
            radius   : spiral radius in meters
            center_x : center of spiral (north)
            center_y : center of spiral (east)
            start_z  : starting Z position (NED, usually 0)
            end_z    : final Z position (NED, negative is upward)
            omega    : angular frequency (rad/s)
        """
        self.R = radius
        self.cx = center_x
        self.cy = center_y
        self.z0 = start_z
        self.zf = end_z
        self.w = omega

    def position_xyz(self, t: float) -> Tuple[float, float, float]:
        """
        Compute 3D reference position at time t.

        x(t) = cx + R * cos(w t)
        y(t) = cy + R * sin(w t)
        z(t) = linear interpolation from start_z to end_z
        """
        x = self.cx + self.R * math.cos(self.w * t)
        y = self.cy + self.R * math.sin(self.w * t)

        # Linear altitude change over one full spiral
        z = self.z0 + (self.zf - self.z0) * (t / self.duration())

        return x, y, z

    def duration(self) -> float:
        """
        Nominal duration for one full spiral revolution.
        """
        return 2 * math.pi / self.w
