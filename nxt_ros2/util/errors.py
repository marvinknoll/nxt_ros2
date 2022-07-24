class Error(Exception):
    """Base class for nxt_ros2 exceptions."""

    pass


class InvalidColorCode(Error):
    """Raised when trying to convert an invalid color code to rgba."""

    def __init__(self):
        self.message = "Invalid color code! valid codes: (1-6)"
        super().__init__(self.message)


class InvalidSensorConfigParams(Error):
    """Raised if invalid sensor param was read or a requried one is missing."""

    def __init__(self, port_str, required_sensor_params):
        self.message = (
            "Missing or invalid sensor config parameters for sensor '%s',"
            " required: %s" % (port_str, required_sensor_params)
        )
        super().__init__(self.message)


class InvalidSensorPort(Error):
    """Raised when the config parameters contain an invalid sensor port."""

    def __init__(self, port, valid_ports):
        self.message = "Invalid sensor port '%s', valid ones: '%s'" % (
            port,
            valid_ports,
        )
        super().__init__(self.message)


class DuplicateSensorPort(Error):
    """Raised when the config parameters contain a duplicate sensor port."""

    def __init__(self, sensor_port):
        self.message = (
            "Duplicate sensor_port '%s' in config parameters" % sensor_port
        )
        super().__init__(self.message)


class DuplicateSensorName(Error):
    """Raised when the config parameters contain a duplicate sensor name."""

    def __init__(self, sensor_name):
        self.message = (
            "Duplicate sensor_name '%s' in config parameters" % sensor_name
        )
        super().__init__(self.message)


class InvalidSensorType(Error):
    """Raised when the config parameters contain an invalid sensor type."""

    def __init__(self, sensor_type, sensor_port, valid_sensor_types):
        self.message = (
            "Invalid sensor_type '%s' config for sensor on port: '%s'."
            " Valid sensor_type's: %s"
            % (sensor_type, sensor_port, valid_sensor_types)
        )
        super().__init__(self.message)


class InvalidMotorConfigparams(Error):
    """Raised if invalis motor param wa read or a require one is missing."""

    def __init__(self, port_str, required_motor_params):
        self.message = (
            "Missing or invalid motor config parameters for motor"
            " '%s', required: %s" % (port_str, required_motor_params)
        )
        super().__init__(self.message)


class InvalidMotorPort(Error):
    """Raised when the config parameters contain an invalid motor port."""

    def __init__(self, port, valid_ports):
        self.message = "Invalid motor port '%s', valid ones: '%s'" % (
            port,
            valid_ports,
        )
        super().__init__(self.message)


class DuplicateMotorName(Error):
    """Raised when the config parameters contain a duplicate motor name."""

    def __init__(self, motor_name):
        self.message = (
            "Duplicate motor_name '%s' in config parameters" % motor_name
        )
        super().__init__(self.message)


class DuplicateMotorMimicName(Error):
    """Raised when the config params contain a duplicate motor mimic_name."""

    def __init__(self, motor_mimic_name):
        self.message = (
            "Duplicate motor_mimic_name '%s' in config parameters"
            % motor_mimic_name
        )
        super().__init__(self.message)


class DuplicateMotorPort(Error):
    """Raised when the config parameters contain a duplicate motor port."""

    def __init__(self, motor_port):
        self.message = (
            "Duplicate motor_port '%s' in config parameters" % motor_port
        )
        super().__init__(self.message)


class InvalidMotorType(Error):
    """Raised when the config parameters contain an invalid motor type."""

    def __init__(self, motor_type, motor_port, valid_motor_types):
        self.message = (
            "Invalid config motor_type: '%s' for motor on port: '%s'."
            " Valid motor_type's: %s"
            % (motor_type, motor_port, valid_motor_types)
        )
        super().__init__(self.message)


class InvalidWheelMotorConfig(Error):
    """Raised if only one of 'wheel_motor_l' and 'wheel_motor_r' is defined."""

    def __init__(self, defined_wheel_motor_type):
        if defined_wheel_motor_type == "wheel_motor_l":
            self.message = (
                "If you define a motor with motor_type 'wheel_motor_l', please"
                " also define one with motor_type: 'wheel_motor_r'. Otherwise"
                " config all motor_types as 'other'"
            )
        elif defined_wheel_motor_type == "wheel_motor_r":
            self.message = (
                "If you define a motor with motor_type 'wheel_motor_r', please"
                " also define one with motor_type: 'wheel_motor_l'. Otherwise"
                " config all motor_types as 'other'"
            )
        super().__init__(self.message)


class InvalidPort(Error):
    """Raised if an invalid port is passed as parameter to nxt_ros2_setup."""

    def __init__(self, port):
        self.message = (
            "Invalid device port parameter '%s' for NxtRos2Setup node."
            % (port)
        )
        super().__init__(self.message)
