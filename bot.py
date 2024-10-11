from typing import Tuple

from pygame import Vector2

from .lib import get_path_points
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

    def compute_commands(
        self, next_waypoint: int, position: Transform, velocity: Vector2
    ) -> Tuple:
        path_point = self.points[next_waypoint]
        relative_position = position.inverse() * path_point.pose.p

        if velocity.length() < path_point.target_velocity(
            relative_position.length(), velocity.length()
        ):
            throttle = 1
        else:
            throttle = -1

        if relative_position.as_polar()[1] > 0:
            return throttle, 1
        else:
            return throttle, -1
