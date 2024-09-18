import math
from dataclasses import dataclass

import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np
from pygame import Vector2

from racer.linear_math import Transform, Rotation
from racer.tracks import track1


@dataclass
class PathPoint:
    pose: Transform
    curvature: float


def get_curvature(base: Transform, target: Transform) -> float:
    relative = base.inverse() * target
    distance = relative.p.length()
    angle = relative.M.angle.real
    return angle / distance if distance > 0 else 0


def get_path_points(points: list[Vector2]) -> list[PathPoint]:
    poses = []
    for i in range(len(points)):
        p = points[i]
        p2 = points[(i + 1) % len(points)]
        angle = math.atan2(p2.y - p.y, p2.x - p.x)
        poses.append(Transform(Rotation.fromangle(angle), p))

    path_points = []
    for i in range(len(poses)):
        p = poses[i]
        p2 = poses[(i + 1) % len(poses)]
        curvature = get_curvature(p, p2)
        path_points.append(PathPoint(pose=p, curvature=curvature))

    return path_points


if __name__ == '__main__':
    points = [Vector2(x, y) for x, y in track1.lines]

    # Perform the interpolation
    path_points = get_path_points(points)

    # Plot the original points and the smooth spline
    plt.figure()
    plt.plot([p.x for p in points], [p.y for p in points], 'o', label='Original Points', markersize=1)

    cmap = plt.cm.jet
    curvatures = np.array([abs(p.curvature) for p in path_points])
    cNorm = colors.Normalize(vmin=np.min(curvatures), vmax=np.max(curvatures))
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cmap)

    # Draw heading vectors
    for p in path_points:
        x = p.pose.p.x
        y = p.pose.p.y
        dx = math.cos(p.pose.M.angle) * 20
        dy = math.sin(p.pose.M.angle) * 20
        colorVal = scalarMap.to_rgba(abs(p.curvature))
        print(abs(p.curvature), colorVal)

        plt.arrow(x, y, dx, dy, head_width=10, head_length=10, color=colorVal)

    plt.legend()
    plt.title('Interpolated')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')  # Ensure equal scaling for both axes
    plt.grid(True)
    plt.show()
