# LEGO type:standard slot:0 autostart

from spike import ColorSensor, MotionSensor, Motor, MotorPair


class Robot:
    def __init__(
        self,
        drive_motor_rotation_ratio: float,
        left_motor_port: str,
        right_motor_port: str,
        left_color_sensor_port: str,
        right_color_sensor_port: str,
        front_attachment_motor_port: str,
        back_attachment_motor_port: str,
    ) -> None:
        self._motion_sensor = MotionSensor()
        self._drive_motors = MotorPair(left_motor_port, right_motor_port)
        self._drive_motor_rotation_ratio = drive_motor_rotation_ratio
        self._drive_motors.set_motor_rotation(drive_motor_rotation_ratio)
        self._color_sensor_left = ColorSensor(left_color_sensor_port)
        self._left_drive_motor = Motor(left_motor_port)
        self._right_drive_motor = Motor(right_motor_port)
        self._left_color_sensor = ColorSensor(left_color_sensor_port)
        self._right_color_sensor = ColorSensor(right_color_sensor_port)
        self._front_attachment_motor = Motor(front_attachment_motor_port)
        self._back_attachment_motor = Motor(back_attachment_motor_port)

    @property
    def drive_motors(self):
        return self._drive_motors

    def move(self, distance_cm: float, speed: int = 75) -> None:
        """Move straight for gyro sensor

        Args:
            distance_cm (float): The distance in cm
            speed (int, optional): The motor speed. Defaults to 75.
        """
        total_degrees = abs(distance_cm) * 360 / self._drive_motor_rotation_ratio
        self._motion_sensor.reset_yaw_angle()
        self._left_drive_motor.get_position()
        motor = self._left_drive_motor if (distance_cm < 0) else self._right_drive_motor
        motor.set_degrees_counted(0)
        speed = speed if (distance_cm > 0) else -speed
        while True:
            degrees_counted = motor.get_degrees_counted()
            if degrees_counted >= total_degrees:
                break
            correction = 0 - self._motion_sensor.get_yaw_angle()
            correction = correction if (distance_cm > 0) else -correction
            self._drive_motors.start(correction, round(speed))
        self._drive_motors.stop()

    def turn(self, degrees: float = 0, speed: int = 10) -> None:
        if degrees < -179 or degrees > 179:
            print("Turn angle must be between -179 and 179")
            return
        self._motion_sensor.reset_yaw_angle()
        steering = 100 if degrees > 0 else -100
        self._drive_motors.start(steering, speed)
        while abs(self._motion_sensor.get_yaw_angle()) < abs(degrees):
            pass
        self._drive_motors.stop()

    def follow_line(
        self, sensor: str, distance_cm: float = 0, to_intersection: bool = False, power: int = 35
    ) -> None:
        total_degrees = round(abs(distance_cm) * 360 / self._drive_motor_rotation_ratio)

        # Determine the color sensor to use for line following
        is_left_color_sensor = True if sensor.lower() == "left" else False

        color_sensor = (
            self._left_color_sensor if is_left_color_sensor else self._right_color_sensor
        )
        stop_color_sensor = (
            self._right_color_sensor if is_left_color_sensor else self._left_color_sensor
        )

        # PID Line Follower
        # https://primelessons.org/en/ProgrammingLessons/PIDLineFollower.pdf
        integral = 0
        last_error = 0
        pfix_factor = 0.3
        lfix_factor = 0.001
        dfix_factor = 1
        is_running = True
        # Reset motor degrees counter
        motor = self._right_drive_motor
        motor.set_degrees_counted(0)
        ramp_power = 10.0
        while is_running:
            error = color_sensor.get_reflected_light() - 50
            pfix = error * pfix_factor
            integral = integral + error
            lfix = integral * lfix_factor
            derivative = error - last_error
            last_error = error
            dfix = derivative * dfix_factor
            correction = round(pfix + lfix + dfix)
            correction = correction if is_left_color_sensor else -correction
            ramp_power = float(power) if ramp_power >= power else ramp_power + 0.1
            self._drive_motors.start_tank_at_power(
                round(ramp_power + correction), round(ramp_power - correction)
            )
            if to_intersection:
                if stop_color_sensor.get_color() == "black":
                    is_running = False
            elif motor.get_degrees_counted() >= total_degrees:
                is_running = False
        self._drive_motors.stop()


LARGE_SPIKE_PRIME_WHEEL = 27.6  # Circumference in cm

robot = Robot(
    drive_motor_rotation_ratio=LARGE_SPIKE_PRIME_WHEEL,
    left_motor_port="A",
    right_motor_port="E",
    left_color_sensor_port="B",
    right_color_sensor_port="F",
    front_attachment_motor_port="D",
    back_attachment_motor_port="C",
)

robot.follow_line(sensor="left", distance_cm=15)
robot.follow_line(sensor="left", to_intersection=True)
robot.move(distance_cm=7, speed=10)
robot.turn(degrees=85)
robot.follow_line(sensor="left", to_intersection=True)
robot.move(distance_cm=3, speed=10)
robot.follow_line(sensor="left", to_intersection=True)
