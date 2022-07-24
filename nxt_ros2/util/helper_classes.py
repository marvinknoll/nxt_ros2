from typing import List

"""Helper classes for motor/sensor configs and robot dimensions.

  Classes:

    SensorConfigs: holds configuration of one or multiple sensors
    MotorConfigs: holds configuration of one or multiple motors
    RobotDimensions: holds robot dimensions (axle_track, wheel_radius,
                    _rad_per_s_to_effort)
"""


class SensorConfigs:
    """
    A class to represent sensor configurations.

    All attributes in this class should have the same length.
    This is the only way to uniquely associate the sensor name with the correct
    properties.

    Attributes
    ----------
    sensor_names : List[str]
        unique names of sensors
    sensor_types : List[str]
        types of sensors ('touch', 'reflected_light', 'color', 'ultrasonic')
    sensor_ports : List[str]
        potrs of sensors ('1', '2', '3' or '4')
    sensor_frame_ids : List[str, None]
        unique frame id's of sensors, or None

    """

    def __init__(self):
        """Initialize an empty SensorConfigs object."""
        self.sensor_names: List[str] = []
        self.sensor_types: List[str] = []
        self.sensor_ports: List[str] = []
        self.sensor_frame_ids: List[str, None] = []


class MotorConfigs:
    """
    A class to represent motor configurations.

    All attributes in this class should have the same length.
    This is the only way to uniquely associate the motor name with the correct
    properties.

    Attributes
    ----------
    motor_ports : List[str]
        unique ports of motors
    motor_names : List[str]
        unique names of motors
    motor_types : List[str]
        types of motors ('other', 'wheel_motor_l', 'wheel_motor_r')
    motor_mimic_names: : List[str]
        unique names of motor mimics
    motor_mimic_gear_ratios : List[str]
        gear ratios between the motor mimic joint and the actual motor
    inver_directions : List[bool]
        inverts the motors direction, used to configure motor orientation,
        check readme for more details

    """

    def __init__(self):
        """Initialize an empty MotorConfigs object."""
        self.motor_ports: List[str] = []
        self.motor_names: List[str] = []
        self.motor_types: List[str] = []
        self.motor_mimic_names: List[str] = []
        self.motor_mimic_gear_ratios: List[float] = []
        self.invert_directions: List[bool] = []


class RobotDimensions:
    """
    A class to represent robot dimensions.

    These dimensions can be used, for example, for drive controllers or
    odometry calculations.

    Attributes
    ----------
    axle_track : float
        distance between wheel_motor_l and wheel_motor_r in meters
    wheel_radius : float
        wheel radius in meter
    rad_per_s_to_effort : float
        multiplier

    """

    def __init__(
        self,
        axle_track: float = -1.0,
        wheel_radius: float = -1.0,
        rad_per_s_to_effort: float = -1.0,
    ):
        """
        Construct all necessary attributes for the RobotDimensions object.

        Parameters
        ----------
        axle_track : float
            distance between wheel_motor_l and wheel_motor_r in meters
        wheel_radius : float
            wheel radius in meter
        rad_per_s_to_effort : float
            multiplier

        """
        self.axle_track: float = axle_track
        self.wheel_radius: float = wheel_radius
        self.rad_per_s_to_effort: float = rad_per_s_to_effort
