#!/usr/bin/env python

# HC_SR04.py
# 2015-08-06
# Public Domain
import os
import time

import pigpio


class sensor:

    def __init__(self, pi, trigger, echo):

        self._pi = pi
        self._trig = trigger
        self._echo = echo

        self._tick = None
        self._distance = 0.0
        self._new = False

        pi.set_mode(self._trig, pigpio.OUTPUT)
        pi.set_mode(self._echo, pigpio.INPUT)

        self._cb = pi.callback(self._echo, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):

        if level == 1:
            self._tick = tick
        else:
            if self._tick is not None:
                ping = pigpio.tickDiff(self._tick, tick)
                self._distance = ping * 17015.0 / 1000000.0
                self._new = True

    def trigger(self):

        self._tick = None
        self._pi.gpio_trigger(self._trig, 10)

    def get_centimetres(self):

        new = self._new
        self._new = False
        return self._distance, new

    def cancel(self):

        self._cb.cancel()


class UltraSonicSensors:
    def __init__(self, pi, sensors_pins):
        self.range_sensors = []
        for i in sensors_pins:
            self.range_sensors.append(sensor(pi, trigger=i[0], echo=i[1]))

        self.sensor_count = len(self.range_sensors)

    def update(self):
        temp = float('Inf')
        for rs in self.range_sensors:
            rs.trigger()
            time.sleep(0.01)
            distance, is_new = rs.get_centimetres()
            if distance < temp:
                temp = distance
        return temp

    def cancel(self):
        for rs in self.range_sensors:
            try:
                rs.cancel()
            except Exception as e:
                print(e)


if __name__ == "__main__":

    pi = pigpio.pi()

    r_sensors_pins = {
        # R
        (23, 24),
        (23, 22),
        (23, 27),
    }
    f_sensors_pins = {
        # F
        (23, 17),
        (23, 4),
    }

    l_sensors_pins = {
        # L
        (23, 18),
        (23, 25),
        (23, 12),

    }

    l_range_sensors = UltraSonicSensors(pi, l_sensors_pins)
    f_range_sensors = UltraSonicSensors(pi, f_sensors_pins)
    r_range_sensors = UltraSonicSensors(pi, r_sensors_pins)

    # sonar = sensor(pi, trigger=23, echo=24)

    end = time.time() + 10.0

    r = 1
    while time.time() < end:
        os.system('clear')
        # time.sleep(0.1)

        print('Min L: ' + str(l_range_sensors.update()))
        print('Min F: ' + str(f_range_sensors.update()))
        print('Min R: ' + str(r_range_sensors.update()))
        time.sleep(0.1)

        # sonar.trigger()
        # cms, new = sonar.get_centimetres()
        # print("{} {:.1f} {}".format(r, cms, new))
        # r += 1

    # sonar.cancel()

    pi.stop()

# import time
#
# import RPi.GPIO as GPIO
#
#
# class RangeSensor:
#
#     def __init__(self, trig=23, echo=24):
#         GPIO.setmode(GPIO.BCM)
#         self.TRIG = trig
#         self.ECHO = echo
#         GPIO.setup(self.TRIG, GPIO.OUT)
#         GPIO.setup(self.ECHO, GPIO.IN)
#         GPIO.output(self.TRIG, False)
#         print("Waiting For Sensor To Settle")
#         time.sleep(2)
#
#     def update(self):
#         time.sleep(0.1)
#         GPIO.output(self.TRIG, True)
#         time.sleep(0.00001)
#         GPIO.output(self.TRIG, False)
#         pulse_start = 0
#         pulse_end = 10
#         while GPIO.input(self.ECHO) == 0:
#             pulse_start = time.time()
#
#         while GPIO.input(self.ECHO) == 1:
#             pulse_end = time.time()
#         pulse_duration = pulse_end - pulse_start
#         distance = pulse_duration * 17150
#
#         return round(distance, 2)
