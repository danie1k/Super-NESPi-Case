#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import os
import time
import sys

import ConfigParser
import RPi.GPIO as GPIO

PWD = os.path.dirname(os.path.realpath(__file__))
TEMPERATURE_ACCURACY = 3
PWM_ACCURACY = 1

PWM_OFF = 100
PWM_ON = 0


class Configuration(object):
    @property
    def fan_pin(self):
        return self._int('FanPin')

    @property
    def sleep_time(self):
        return self._float('SleepTime')

    @property
    def pwm_frequency(self):
        return self._int('PwmFrequency')

    @property
    def pwm_steps(self):
        return self._int('PwmSteps')

    @property
    def fan_temperature_on(self):
        return self._float('FanTemperatureOn', TEMPERATURE_ACCURACY)

    @property
    def fan_temperature_full_speed(self):
        return self._float('FanTemperatureFullSpeed', TEMPERATURE_ACCURACY)

    @property
    def fan_speed_min(self):
        return self._int('FanSpeedMin')

    @property
    def fan_speed_max(self):
        return self._int('FanSpeedMax')

    def __init__(self):
        self._config = ConfigParser.SafeConfigParser()
        self._config.read(os.path.join(PWD, 'settings.ini'))

    def get_steps(self):
        temperature_step = float(self.fan_temperature_full_speed - self.fan_temperature_on) / self.pwm_steps
        pwm_step = float(self.fan_speed_max - self.fan_speed_min)/self.pwm_steps

        temperatures = (self.fan_temperature_on + (i * temperature_step) for i in range(1, self.pwm_steps + 1))
        pwms = (self.fan_speed_min + (i*pwm_step) for i in range(1, self.pwm_steps + 1))

        return tuple(zip(
            [round(item, TEMPERATURE_ACCURACY) for item in temperatures],
            [round(item, PWM_ACCURACY) for item in list(pwms)[::-1]],
        ))

    def _float(self, option, rounding=2):
        return round(float(self._config.get('Fan', option)), rounding)

    def _int(self, option):
        return int(self._config.get('Fan', option))


class FanController(object):
    def __init__(self):
        self.configuration = Configuration()
        self.steps = self.configuration.get_steps()
        self._init_gpio()

    def start(self):
        print('Starting fan controller')
        self.fan.start(PWM_OFF)

        while True:
            cpu_temperature = self._get_temperature()
            pwm_to_set = PWM_OFF

            if cpu_temperature > self.configuration.fan_temperature_on:
                # Looking for lowest pwm for current CPU temperature
                for step_temperature, step_pwm in self.steps:
                    pwm_to_set = step_pwm
                    if step_temperature >= cpu_temperature:
                        break

            print('Temperature: {} PWM: {}'.format(cpu_temperature, pwm_to_set))
            self.fan.ChangeDutyCycle(pwm_to_set)
            time.sleep(self.configuration.sleep_time)

    def stop(self):
        print('Stopping fan controller')
        GPIO.cleanup()

    def _get_temperature(self):
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as fp:
            return round(float(fp.read()) / 1000, TEMPERATURE_ACCURACY)

    def _init_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.configuration.fan_pin, GPIO.OUT, initial=GPIO.HIGH)
        self.fan = GPIO.PWM(self.configuration.fan_pin, self.configuration.pwm_frequency)


if __name__ == '__main__':
    controller = FanController()
    try:
        controller.start()
    except:
        controller.stop()
        sys.exit()
