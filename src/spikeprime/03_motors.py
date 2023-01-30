# LEGO type:standard slot:0 autostart

from spike import MotorPair, PrimeHub

hub = PrimeHub()

LARGE_SPIKE_PRIME_WHEEL_CIRCUMFERENCE = 27.6  # cm
SMALL_SPIKE_PRIME_WHEEL_CIRCUMFERENCE = 17.5  # cm

motor_pair = MotorPair("E", "F")
motor_pair.set_motor_rotation(LARGE_SPIKE_PRIME_WHEEL_CIRCUMFERENCE, "cm")


# motor_pair.move(27.6, 'cm', steering=0, speed=10)


# motor_pair.move(9.5 * math.pi / 2, 'cm', steering=100, speed=10)

motor_pair.start(steering=100, speed=10)
