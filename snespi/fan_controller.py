#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import warnings
from _snespi import SNESPi

warnings.simplefilter('ignore')
from simple_pid import PID  # noqa: E402 pylint:disable=wrong-import-position,wrong-import-order

# Base constants
GPIO_PINS = (12, 32, 33)
PWM_MIN, PWM_FULL = 0, 100
REAL_PWM_100, REAL_PWM_0 = 0, 100

# Read configuration from ini file(s)
settings = SNESPi.read_settings()

FAN_TEMPERATURE_ON = settings.getint('Fan', 'temperature_on')
FAN_SPEED_MIN_PWM = settings.getint('Fan', 'speed_min_pwm')
FAN_SPEED_MAX_PWM = settings.getint('Fan', 'speed_max_pwm')
PID_KP = settings.getfloat('Fan', 'pid_kp')
PID_KI = settings.getfloat('Fan', 'pid_ki')
PID_KD = settings.getfloat('Fan', 'pid_kd')
FAN_GPIO_PIN = settings.getint('Fan', 'gpio_pin')
PWM_FREQUENCY = settings.getint('Fan', 'pwm_frequency')


class FanController(SNESPi):
    DEFAULT_SIMULATION_TEMP_MIN = FAN_TEMPERATURE_ON
    DEFAULT_SIMULATION_TEMP_MAX = int(round(FAN_TEMPERATURE_ON * 1.5))
    DEFAULT_SIMULATION_CYCLES = 1

    _controller_name_ = 'Fan Controller'

    _simulator_args_ = (
        SNESPi.SimulatorArg(
            option_string='--temp-min', type=int, default=DEFAULT_SIMULATION_TEMP_MIN,
            help='[°C] Min temperature in cycle',
        ),
        SNESPi.SimulatorArg(
            option_string='--temp-max', type=int, default=DEFAULT_SIMULATION_TEMP_MAX,
            help='[°C] Max temperature in cycle',
        ),
        SNESPi.SimulatorArg(
            option_string='--cycles', type=int, default=DEFAULT_SIMULATION_CYCLES,
            help='Number of repeat cycles of simulation',
        ),
        SNESPi.SimulatorArg(
            option_string='--temperature-on', type=int, default=FAN_TEMPERATURE_ON,
            help='[°C] Temperature at which the fan will be turned on',
        ),
        SNESPi.SimulatorArg(
            option_string='--speed-min-pwm', type=int, default=FAN_SPEED_MIN_PWM,
            help='[0-100%%] Minimum fan speed',
        ),
        SNESPi.SimulatorArg(
            option_string='--speed-max-pwm', type=int, default=FAN_SPEED_MAX_PWM,
            help='[0-100%%] Maximimum fan speed',
        ),
        SNESPi.SimulatorArg(
            option_string='--pid-kp', type=float, default=PID_KP,
            help='The value for the proportional gain Kp of PID Controller',
        ),
        SNESPi.SimulatorArg(
            option_string='--pid-ki', type=float, default=PID_KI,
            help='The value for the integral gain Ki of PID Controller',
        ),
        SNESPi.SimulatorArg(
            option_string='--pid-kd', type=float, default=PID_KD,
            help='The value for the derivative gain Kd of PID Controller',
        ),
        SNESPi.SimulatorArg(
            option_string='--pwm-frequency', type=float, default=PWM_FREQUENCY,
            help='[Hz] PWM switching frequency',
        ),
    )
    _simulator_help_ = (
        'Simulation will REALLY RUN your fan with given settings. '
        'This mode generates fake temperature reads in given number of \'cycles\'.'
    )

    fan = None
    pid = None

    def loop_init(self):
        self.pid = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            setpoint=-1 * FAN_TEMPERATURE_ON,
            sample_time=self.sleep_time,
            output_limits=(FAN_SPEED_MIN_PWM, FAN_SPEED_MAX_PWM),
        )
        if self.GPIO:
            self.GPIO.setup(FAN_GPIO_PIN, self.GPIO.OUT, initial=self.GPIO.HIGH)
            self.fan = self.GPIO.PWM(FAN_GPIO_PIN, PWM_FREQUENCY)
            self.fan.start(REAL_PWM_0)

    def loop_tick(self):
        cpu_temperature = self._get_temperature()
        duty_cycle = self.pid(-1 * cpu_temperature)
        duty_cycle = PWM_MIN if duty_cycle == FAN_SPEED_MIN_PWM else duty_cycle

        fan_speed = '%d%%' % duty_cycle
        duty_cycle = PWM_FULL - duty_cycle  # Reversing PWM, because 0 = fan full speed / 100 = fan stopped

        if self.verbose:
            print 'Temperature: %d°C : Fan Speed: %s' % (cpu_temperature, fan_speed)

        if self.GPIO:
            self.fan.ChangeDutyCycle(duty_cycle)

    def prepare_simulator(self, **parser_options):
        globals()['FAN_TEMPERATURE_ON'] = parser_options['temperature_on']
        globals()['FAN_SPEED_MIN_PWM'] = parser_options['speed_min_pwm']
        globals()['FAN_SPEED_MAX_PWM'] = parser_options['speed_max_pwm']
        globals()['PID_KP'] = parser_options['pid_kp']
        globals()['PID_KI'] = parser_options['pid_ki']
        globals()['PID_KD'] = parser_options['pid_kd']
        globals()['PWM_FREQUENCY'] = parser_options['pwm_frequency']

        temp_min = parser_options['temp_min']
        temp_max = parser_options['temp_max']
        cycles = parser_options['cycles']

        from mock import Mock
        temperatures = list(range(temp_min, temp_max))
        temperatures = temperatures + temperatures[::-1]
        self._get_temperature = Mock(side_effect=temperatures * cycles)

    def validate_configuration(self):
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

    def _get_temperature(self):  # pylint:disable=method-hidden
        if not self.GPIO:
            return 50
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as fp:
            return round(float(fp.read()) / 1000)


if __name__ == '__main__':
    FanController.from_argparser()
