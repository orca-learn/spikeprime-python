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
        """Initialize robot

        This is the initialization method for a robot class. It takes in several parameters such
        as the drive motor rotation ratio, motor and color sensor ports, and attachment motor
        ports. It initializes various sensors and motors for the robot, such as the motion sensor,
        motor pair, color sensors, and attachment motors. The drive motor rotation ratio is set
        for the motor pair.

        Args:
            drive_motor_rotation_ratio (float): _description_
            left_motor_port (str): _description_
            right_motor_port (str): _description_
            left_color_sensor_port (str): _description_
            right_color_sensor_port (str): _description_
            front_attachment_motor_port (str): _description_
            back_attachment_motor_port (str): _description_
        """
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
    def drive_motors(self) -> Motor:
        return self._drive_motors

    def move(self, distance_cm: float, speed: int = 75) -> None:
        """Command the robot to move

        This function allows the robot to move straight using the gyro sensor. The distance to be
        traveled in centimeters and the motor speed can be specified as arguments. The function
        calculates the total degrees the motor needs to turn based on the distance and the motor's
        rotation ratio. It then resets the yaw angle of the motion sensor and sets the degrees
        counted by the left motor to zero. The motor to be used is determined based on the
        direction of movement. The function continuously checks the degrees counted by the motor
        and corrects the robot's direction using the motion sensor's yaw angle until the total
        degrees are reached. Finally, the motors are stopped.

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
        """Command the robot to turn

        This is a method that turns the robot a certain number of degrees at a given speed.
        The method checks if the turn angle is within the valid range of -179 to 179 degrees.
        If it is, the method resets the yaw angle sensor, sets the steering direction based on
        the turn angle, and starts the drive motors at the given speed. The method then waits
        until the robot has turned the desired number of degrees before stopping the drive motors.
        If the turn angle is not within the valid range, the method prints an error message and
        returns without performing the turn.

        Args:
            degrees (float, optional): The number of degrees to turn. Defaults to 0.
            speed (int, optional): The speed of the drive motor. Defaults to 10.
        """
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
        """Command the robot to follow a line

        This function allows the robot to follow a line using a PID line follower algorithm. It
        takes in the sensor to use for line following, the distance to travel in centimeters, a
        boolean indicating whether to stop at an intersection or not, and the power to use for the
        drive motors. The algorithm calculates the necessary correction to keep the robot on the
        line and adjusts the power of the motors accordingly. If to_intersection is True, the
        robot will stop at a black line detected by the stop_color_sensor. Otherwise, it will
        stop once it has traveled the specified distance.

        Args:
            sensor (str): The sensor to use for line following
            distance_cm (float, optional): The distance to travel in centimeters. Defaults to 0.
            to_intersection (bool, optional): whether to stop at an intersection or not. Defaults to False.
            power (int, optional): the power to use for the drive motors. Defaults to 35.
        """
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
