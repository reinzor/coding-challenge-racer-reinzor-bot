import math
import sys
from dataclasses import dataclass
from typing import List

import numpy as np
from pygame import Vector2

from racer.linear_math import Transform, Rotation
from racer.tracks import track1

CURVATURE_CLUSTERING_DISTANCE = 400.
CURVATURE_VELOCITY_SCALING = 1.9
MAX_VELOCITY = 300.

DISTANCE_OFFSET = 50.
ACCELERATION = 80.

DEBUG = "--reinzor" in sys.argv


def acceleration_velocity_profile(distance: float, velocity_offset: float, acceleration=80.,
                                  distance_offset: float = 50.) -> float:
    d = max(0., distance - distance_offset)
    return math.sqrt(2 * acceleration * d) + velocity_offset


def distance_velocity_function(distance: float, velocity_offset: float) -> float:
    if distance < 0.:
        raise ValueError(f"Distance {distance} cannot be negative")
    return acceleration_velocity_profile(distance, velocity_offset, ACCELERATION, DISTANCE_OFFSET)


@dataclass
class PathPoint:
    pose: Transform
    curvature: float
    max_velocity: float
    curvature_velocity_scaling: float

    @property
    def curvature_velocity(self):
        if self.curvature == 0:
            return self.max_velocity
        return self.curvature_velocity_scaling / abs(self.curvature)

    def target_velocity(self, distance: float, velocity: float) -> float:
        target_velocity = min(self.max_velocity, distance_velocity_function(distance, self.curvature_velocity))
        if DEBUG:
            print(
                f"Curvature velocity: {self.curvature_velocity:.01f}, target velocity: {target_velocity:.01f}, velocity: {velocity:.01f}")
        return target_velocity


def get_curvature(base: Transform, target: Transform) -> float:
    relative = base.inverse() * target
    distance = relative.p.length()
    angle = relative.M.angle.real
    return angle / distance if distance > 0 else 0


def get_path_points(points: List[Vector2]) -> List[PathPoint]:
    poses = []
    for i in range(len(points)):
        p = points[i]
        p2 = points[(i + 1) % len(points)]
        angle = math.atan2(p2.y - p.y, p2.x - p.x)
        poses.append(Transform(Rotation.fromangle(angle), p))

    path_points = []
    i = 0
    while i < len(poses):
        p = poses[i]
        max_curvature = 0.
        j = i
        while j < i + len(poses):
            j += 1
            p2 = poses[j % len(poses)]
            curvature = get_curvature(p, p2)
            if p.p.distance_to(p2.p) > CURVATURE_CLUSTERING_DISTANCE:
                break
            if abs(curvature) > abs(max_curvature):
                max_curvature = curvature
        for k in range(i, j):
            path_points.append(PathPoint(poses[k % len(poses)], max_curvature, MAX_VELOCITY, CURVATURE_VELOCITY_SCALING))
        i = j

    return path_points


if __name__ == '__main__':
    import matplotlib.cm as cmx
    import matplotlib.colors as colors
    import matplotlib.pyplot as plt

    points = [Vector2(x, y) for x, y in track1.lines]

    # Perform the interpolation
    path_points = get_path_points(points)

    # Plot the original points and the smooth spline
    plt.figure()
    plt.plot([p.x for p in points], [p.y for p in points], 'o', label='Original Points', markersize=1)

    # Create a red color map with intensity based on the velocity
    curvature_velocities = np.array([p.curvature_velocity for p in path_points])
    cNorm = colors.Normalize(vmin=np.min(curvature_velocities), vmax=np.max(curvature_velocities))
    cmap = plt.get_cmap('jet')
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cmap)

    # Draw heading vectors
    for p in path_points:
        x = p.pose.p.x
        y = p.pose.p.y
        dx = math.cos(p.pose.M.angle) * 20
        dy = math.sin(p.pose.M.angle) * 20
        colorVal = scalarMap.to_rgba(p.curvature_velocity)
        plt.arrow(x, y, dx, dy, head_width=10, head_length=10, color=colorVal)

    plt.legend()
    plt.title('Interpolated')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')  # Ensure equal scaling for both axes
    plt.grid(True)
    plt.show()
