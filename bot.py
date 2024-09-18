from typing import Tuple

from pygame import Vector2

from .path_processing import get_path_points
from .velocity_function import distance_velocity_function
from ...bot import Bot
from ...linear_math import Transform
from ...track import Track


class ReinzorBot(Bot):
    def __init__(self, track: Track):
        super().__init__(track)
        self.points = get_path_points(track.lines)

    @property
    def name(self):
        return "reinzor"

    @property
    def contributor(self):
        return "Rein"

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        point = self.points[next_waypoint]
        # calculate the target in the frame of the robot
        relative_point = position.inverse() * point.pose.p
        # calculate the angle to the target
        angle = relative_point.as_polar()[1]

        # calculate the throttle
        curvature_velocity = abs(1. / point.curvature) * 0.75
        target_velocity = distance_velocity_function(relative_point.length(), 60, 0., 40, curvature_velocity)
        if velocity.length() < target_velocity:
            throttle = 1
        else:
            throttle = -1

        # calculate the steering
        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1
