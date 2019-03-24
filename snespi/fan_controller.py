#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import argparse
import os
import time
import sys
import warnings

from ConfigParser import SafeConfigParser

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    print '---------------------------------------------------------------'
    print 'Running outside Raspberry Pi, GPIO functions has been disabled!'
    print '---------------------------------------------------------------'
    GPIO = False

warnings.simplefilter('ignore')
from simple_pid import PID  # noqa: E402  # pylint:disable=wrong-import-position

# Base constants

PWD = os.path.dirname(os.path.realpath(__file__))
GPIO_PINS = (12, 32, 33)
PWM_MIN, PWM_FULL = 0, 100
REAL_PWM_100, REAL_PWM_0 = 0, 100

# Read configuration from ini file(s)

parser = SafeConfigParser()
parser.read([
    os.path.join(PWD, 'settings.ini'),
    os.path.join(PWD, 'settings.local.ini'),
])

SLEEP_TIME = parser.getint('General', 'sleep_time')

FAN_TEMPERATURE_ON = parser.getint('Fan', 'temperature_on')
FAN_SPEED_MIN_PWM = parser.getint('Fan', 'speed_min_pwm')
FAN_SPEED_MAX_PWM = parser.getint('Fan', 'speed_max_pwm')

PID_KP = parser.getfloat('Fan', 'pid_kp')
PID_KI = parser.getfloat('Fan', 'pid_ki')
PID_KD = parser.getfloat('Fan', 'pid_kd')

FAN_GPIO_PIN = parser.getint('Fan', 'gpio_pin')
PWM_FREQUENCY = parser.getint('Fan', 'pwm_frequency')

DEFAULT_SIMULATION_TEMP_MIN = FAN_TEMPERATURE_ON
DEFAULT_SIMULATION_TEMP_MAX = int(round(FAN_TEMPERATURE_ON * 1.5))
DEFAULT_SIMULATION_CYCLES = 1


class FanController(object):
    def __init__(self, verbose=None):
        self._verify_config()

        self.verbose = verbose or False

        self.fan = None
        self.pid = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            setpoint=-1 * FAN_TEMPERATURE_ON,
            sample_time=SLEEP_TIME,
            output_limits=(FAN_SPEED_MIN_PWM, FAN_SPEED_MAX_PWM),
        )
        if GPIO:
            self._init_gpio()

    def start(self):
        if self.verbose:
            print 'Starting fan controller'
        if GPIO:
            self.fan.start(REAL_PWM_0)

        while True:
            cpu_temperature = self._get_temperature()
            duty_cycle = self.pid(-1 * cpu_temperature)

            if duty_cycle == FAN_SPEED_MIN_PWM:
                duty_cycle = PWM_MIN

            fan_speed = '%d%%' % duty_cycle
            # Reversing PWM, because 0 = fan full speed / 100 - fan stopped
            duty_cycle = 100 - duty_cycle

            if self.verbose:
                print 'Temperature: %d°C : Fan Speed: %s' % (cpu_temperature, fan_speed)

            if GPIO:
                self.fan.ChangeDutyCycle(duty_cycle)

            time.sleep(SLEEP_TIME)

    def simulate(self, temp_min=None, temp_max=None, cycles=None):
        print '********** Running simulation mode **********'

        temp_min = temp_min or DEFAULT_SIMULATION_TEMP_MIN
        temp_max = temp_max or DEFAULT_SIMULATION_TEMP_MAX
        cycles = cycles or DEFAULT_SIMULATION_CYCLES

        from mock import Mock
        temperatures = list(range(temp_min, temp_max))
        temperatures = temperatures + temperatures[::-1]
        self._get_temperature = Mock(side_effect=temperatures * cycles)
        self.start()

    def stop(self):
        if self.verbose:
            print 'Stopping fan controller'
        if GPIO:
            self.fan.ChangeDutyCycle(REAL_PWM_0)
            GPIO.cleanup()

    def _get_temperature(self):  # pylint:disable=method-hidden
        if not GPIO:
            return 50
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as fp:
            return round(float(fp.read()) / 1000)

    def _init_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(FAN_GPIO_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.fan = GPIO.PWM(FAN_GPIO_PIN, PWM_FREQUENCY)

    def _verify_config(self):
        assert SLEEP_TIME > 0, 'Setting \'sleep_time\' must be > 0'
        assert FAN_TEMPERATURE_ON > 0, 'Setting \'temperature_on\' must be > 0'
        assert PWM_FREQUENCY > 0, 'Setting \'pwm_frequency\' must be > 0'

        assert PID_KP >= 0, 'Setting \'pid_kp\' must be >= 0'
        assert PID_KI >= 0, 'Setting \'pid_ki\' must be >= 0'
        assert PID_KD >= 0, 'Setting \'pid_kd\' must be >= 0'

        assert PWM_MIN <= FAN_SPEED_MIN_PWM <= PWM_FULL, (
            'Setting \'speed_min_pwm\' must have value between %d - %s' % (PWM_MIN, PWM_FULL)
        )
        assert PWM_MIN <= FAN_SPEED_MAX_PWM <= PWM_FULL, (
            'Setting \'speed_max_pwm\' must have value between %d - %s' % (PWM_MIN, PWM_FULL)
        )
        assert FAN_SPEED_MIN_PWM < FAN_SPEED_MAX_PWM, (
            'Setting \'speed_min_pwm\' must be lower than \'speed_max_pwm\''
        )
        assert FAN_GPIO_PIN in GPIO_PINS, (
            'Setting \'gpio_pin\' must be one of %s' % (GPIO_PINS,)
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Increase output verbosity',
    )

    # Simulation options
    subparsers = parser.add_subparsers()  # pylint:disable=invalid-name
    subparsers.required = False

    simulation = subparsers.add_parser(  # pylint:disable=invalid-name
        'simulation',
        help=(
            'Simulation will REALLY RUN your fan with given settings. '
            'This mode generates fake temperature reads in given number of \'cycles\'. '
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    simulation.add_argument(
        '--temp-min',
        help='[°C] Min temperature in cycle',
        type=int, default=DEFAULT_SIMULATION_TEMP_MIN,
    )
    simulation.add_argument(
        '--temp-max',
        help='[°C] Max temperature in cycle',
        type=int, default=DEFAULT_SIMULATION_TEMP_MAX,
    )
    simulation.add_argument(
        '--cycles',
        help='Number of repeat cycles of simulation',
        type=int, default=DEFAULT_SIMULATION_CYCLES,
    )

    simulation.add_argument(
        '--sleep-time',
        help='[seconds] Time between temperature checks',
        type=int, default=SLEEP_TIME,
    )
    simulation.add_argument(
        '--temperature-on',
        help='[°C] Temperature at which the fan will be turned on',
        type=int, default=FAN_TEMPERATURE_ON,
    )
    simulation.add_argument(
        '--speed-min-pwm',
        help='[0-100%%] Minimum fan speed',
        type=int, default=FAN_SPEED_MIN_PWM,
    )
    simulation.add_argument(
        '--speed-max-pwm',
        help='[0-100%%] Maximimum fan speed',
        type=int, default=FAN_SPEED_MAX_PWM,
    )
    simulation.add_argument(
        '--pid-kp',
        help='The value for the proportional gain Kp of PID Controller',
        type=float, default=PID_KP,
    )
    simulation.add_argument(
        '--pid-ki',
        help='The value for the integral gain Ki of PID Controller',
        type=float, default=PID_KI,
    )
    simulation.add_argument(
        '--pid-kd',
        help='The value for the derivative gain Kd of PID Controller',
        type=float, default=PID_KD,
    )

    simulation.add_argument(
        '--pwm-frequency',
        help='[Hz] PWM switching frequency',
        type=float, default=PWM_FREQUENCY,
    )

    # Because there is no optional subparser in Python 2 - https://bugs.python.org/issue9253
    def _get_options():
        args = sys.argv[1:]
        simulation_mode = 'simulation' in args
        if not simulation_mode:
            args.append('simulation')

        parser_options = parser.parse_args(args=args)
        parser_options.simulation_mode = simulation_mode
        return parser_options

    options = _get_options()  # pylint:disable=invalid-name

    if options.simulation_mode:
        SLEEP_TIME = options.sleep_time
        FAN_TEMPERATURE_ON = options.temperature_on
        FAN_SPEED_MIN_PWM = options.speed_min_pwm
        FAN_SPEED_MAX_PWM = options.speed_max_pwm
        PID_KP = options.pid_kp
        PID_KI = options.pid_ki
        PID_KD = options.pid_kd
        PWM_FREQUENCY = options.pwm_frequency

    controller = FanController(verbose=options.verbose or options.simulation_mode)  # pylint:disable=invalid-name
    try:
        if options.simulation_mode:
            controller.simulate(
                temp_min=options.temp_min,
                temp_max=options.temp_max,
                cycles=options.cycles,
            )
        else:
            controller.start()
    finally:
        controller.stop()
        sys.exit()
