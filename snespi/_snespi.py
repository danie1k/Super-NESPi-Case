# -*- coding: utf-8 -*-
import os
import sys
import time

import argparse
from ConfigParser import SafeConfigParser
from collections import namedtuple

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    from mock import MagicMock
    GPIO = MagicMock()
    print '---------------------------------------------------------------'
    print 'Running outside Raspberry Pi, GPIO functions has been disabled!'
    print '---------------------------------------------------------------'

PWD = os.path.dirname(os.path.realpath(__file__))

# pylint:disable=protected-access


class SNESPi(object):
    _controller_name_ = None
    _simulator_args_ = tuple()
    _simulator_help_ = None

    SimulatorArg = namedtuple('SimulatorArg', 'option_string help type default')

    def loop_init(self):
        raise NotImplementedError

    def loop_tick(self):
        raise NotImplementedError

    def prepare_simulator(self, **parser_options):
        raise NotImplementedError

    def validate_configuration(self):
        raise NotImplementedError

    def __init__(self, sleep_time, verbose):
        self.sleep_time = sleep_time
        self.verbose = verbose
        self.GPIO = GPIO

    def _simulate(self, **kwargs):
        print '********** Running simulation mode **********'
        self.prepare_simulator(**kwargs)
        self._start()

    def _start(self):
        self._init_gpio()

        self.loop_init()

        assert self.sleep_time > 0, 'Setting \'sleep_time\' must be > 0'
        self.validate_configuration()

        if self.verbose:
            print 'Starting %s' % self._controller_name_

        while True:
            self.loop_tick()
            time.sleep(self.sleep_time)

    def _stop(self):
        if self.verbose:
            print 'Stopping %s' % self._controller_name_
        GPIO.cleanup()

    def _init_gpio(self):
        GPIO.setmode(GPIO.BOARD)

    @classmethod
    def from_argparser(cls, **kwargs):
        settings = SNESPi.read_settings()
        sleep_time = settings.getint('General', 'sleep_time')

        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-v', '--verbose',
            action='store_true',
            help='Increase output verbosity',
        )

        # Simulation options
        subparsers = parser.add_subparsers()
        simulation = subparsers.add_parser(
            'simulation',
            help=cls._simulator_help_,
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        simulation.add_argument(
            '--sleep-time',
            help='[seconds] Time between \'loop_tick\' calls',
            type=int, default=sleep_time,
        )

        for argument in cls._simulator_args_:
            argument_kwargs = argument._asdict()
            simulation.add_argument(argument_kwargs.pop('option_string'), **argument_kwargs)

        # Because there is no optional subparser in Python 2 - https://bugs.python.org/issue9253
        args = sys.argv[1:]
        simulation_mode = 'simulation' in args
        if not simulation_mode:
            args.append('simulation')

        parser_options = parser.parse_args(args=args)
        parser_options.simulation_mode = simulation_mode

        controller = cls(
            sleep_time=parser_options.sleep_time,
            verbose=parser_options.verbose or parser_options.simulation_mode,
            **kwargs
        )
        try:
            if parser_options.simulation_mode:
                controller._simulate(**parser_options.__dict__)
            else:
                controller._start()
        except Exception as ex:  # pylint:disable=broad-except
            print 'ERROR: %s ' % ex
            sys.exit()
        finally:
            controller._stop()
            sys.exit()

    @staticmethod
    def read_settings():
        parser = SafeConfigParser()
        parser.read([os.path.join(PWD, 'settings.ini'), os.path.join(PWD, 'settings.local.ini')])
        return parser
