from typing import List


class SensorConfigs:
    def __init__(self):
        self.sensor_names: List[str] = []
        self.sensor_types: List[str] = []
        self.sensor_ports: List[str] = []
        self.sensor_frame_ids: List[str, None] = []


class MotorConfigs:
    def __init__(self):
        self.motor_ports: List[str] = []
        self.motor_names: List[str] = []
        self.motor_types: List[str] = []
        self.motor_mimic_names: List[str] = []
        self.motor_mimic_gear_ratios: List[float] = []
        self.invert_directions: List[bool] = []


class RobotDimensions:
    def __init__(
        self,
        axle_track: float = -1.0,
        wheel_radius: float = -1.0,
        rad_per_s_to_effort: float = -1.0,
    ):
        self.axle_track: float = axle_track
        self.wheel_radius: float = wheel_radius
        self.rad_per_s_to_effort: float = rad_per_s_to_effort
