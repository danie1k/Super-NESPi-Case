#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import os
import subprocess
import time

from _snespi import SNESPi

# Base constants
GPIO_PINS = (2, 5, 7, 8, 10, 11, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 27, 28, 29, 31, 35, 36, 37, 38, 40)
CMD_KILL_EMULATOR = 'kill -QUIT $(ps aux | grep -v grep | grep retropie/emulators | awk \'{print $2}\')'
CMD_RESTART_EMULATIONSTATION = 'touch /tmp/es-restart && killall emulationstation'
CMD_RESTART_CONSOLE = 'reboot'

# Read configuration from ini file(s)
settings = SNESPi.read_settings()

RESET_GPIO_PIN = settings.getint('Reset', 'gpio_pin')
KILL_EMULATOR_PRESS_TIME_LIMIT = settings.getint('Reset', 'kill_emulator_press_time_limit')
RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT = settings.getint('Reset', 'restart_emulationstation_press_time_limit')


class SoftReset(SNESPi):
    DEFAULT_BOUNCE_TIME = 1000

    _controller_name_ = 'Soft Reset Controller'

    _simulator_args_ = (
        SNESPi.SimulatorArg(
            option_string='--kill-emulator-time', type=int, default=KILL_EMULATOR_PRESS_TIME_LIMIT,
            help='[seconds] Reset key press time limit to shut down current emulator',
        ),
        SNESPi.SimulatorArg(
            option_string='--restart-emulationstation-time', type=int,
            default=RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT,
            help='[seconds] Reset key press time limit to restart Emulation Station',
        ),
        SNESPi.SimulatorArg(
            option_string='--bounce-time', type=int, default=DEFAULT_BOUNCE_TIME,
            help='GPIO pin bounce time',
        ),
    )
    _simulator_help_ = (
        'Simulation will REALLY CATCH button presses, but only display messages instead resetting your RPi.'
    )

    def __init__(self, *args, **kwargs):
        super(SoftReset, self).__init__(*args, **kwargs)
        self.bounce_time = self.DEFAULT_BOUNCE_TIME

    def loop_init(self):
        self.GPIO.setup(RESET_GPIO_PIN, self.GPIO.IN)

        def reset_btn_callback(channel):
            press_start = time.time()

            while self.GPIO.input(channel) == 1:
                pass  # Wait while reset button is pressed

            keypress_time = time.time() - press_start

            if keypress_time <= 0.0001:
                return

            if self.verbose:
                print 'Button press time: %.4f second(s)' % keypress_time

            if keypress_time < KILL_EMULATOR_PRESS_TIME_LIMIT:
                self._run_command(CMD_KILL_EMULATOR, 'Killing current emulator')
            elif KILL_EMULATOR_PRESS_TIME_LIMIT <= keypress_time < RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT:
                self._run_command(CMD_RESTART_EMULATIONSTATION, 'Restarting Emulation Station')
            else:
                self._run_command(CMD_RESTART_CONSOLE, 'Restarting Console')

        self.GPIO.add_event_detect(
            RESET_GPIO_PIN,
            self.GPIO.RISING,
            callback=reset_btn_callback,
            bouncetime=self.bounce_time,
        )

    def loop_tick(self):
        if self.verbose:
            print 'Tick'

    def prepare_simulator(self, **parser_options):
        globals()['KILL_EMULATOR_PRESS_TIME_LIMIT'] = parser_options['kill_emulator_time']
        globals()['RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT'] = parser_options['restart_emulationstation_time']
        self.bounce_time = parser_options['bounce_time']

        def mocked_run(cmd, msg):
            print '[Command Simulator] Message: "%s"' % msg
            print '[Command Simulator] Command: "%s"' % cmd

        self._run_command = mocked_run

    def validate_configuration(self):
        assert KILL_EMULATOR_PRESS_TIME_LIMIT > 0, (
            'Setting \'kill_emulator_press_time_limit\' must be > 0'
        )
        assert RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT > 0, (
            'Setting \'restart_emulationstation_press_time_limit\' must be > 0'
        )
        assert KILL_EMULATOR_PRESS_TIME_LIMIT < RESTART_EMULATIONSTATION_PRESS_TIME_LIMIT, (
            'Setting \'kill_emulator_press_time_limit\' must be lower '
            'than \'restart_emulationstation_press_time_limit\''
        )
        assert RESET_GPIO_PIN in GPIO_PINS, (
            'Setting \'gpio_pin\' must be one of %s' % (GPIO_PINS,)
        )

    def _run_command(self, command, message):  # pylint:disable=method-hidden
        print message
        subprocess.Popen(command, shell=True, stdout=open(os.devnull, 'w'), stderr=subprocess.STDOUT)


if __name__ == '__main__':
    SoftReset.from_argparser()
