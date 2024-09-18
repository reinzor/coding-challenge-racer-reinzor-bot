import math


def acceleration_velocity_profile(distance: float, acceleration: float, time_delay: float, distance_offset: float,
                                  velocity_offset: float) -> float:
    d = max(0., distance - distance_offset)
    return math.sqrt(
        pow(acceleration, 2) * pow(time_delay, 2) + 2 * acceleration * d) - acceleration * time_delay + velocity_offset


def distance_velocity_function(distance: float, acceleration: float, time_delay: float, distance_offset: float,
                               velocity_offset: float) -> float:
    if distance < 0.:
        raise ValueError(f"Distance {distance} cannot be negative")
    return acceleration_velocity_profile(distance, acceleration, time_delay, distance_offset, velocity_offset)
