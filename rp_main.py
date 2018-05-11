import threading
import time

import RPi.GPIO as GPIO
import numpy
import pigpio
from mpu6050 import mpu6050

from fuzzy_controller.fuzzy_system import FuzzySystem
from misc.motor_controller import QuadMotorController
from misc.range_sensor import UltraSonicSensors

# init controllers
motor_controller = QuadMotorController()

gyro_sensor = mpu6050(0x68)
pi = pigpio.pi()

l_range_sensors_pins = {(23, 24)}
f_range_sensors_pins = {(23, 24)}
r_range_sensors_pins = {(23, 24)}

run = 'Running'
STOP = 'Stopped'
MANUAL = 'Manual'

status = run

goal_threshold = 0.5
# position
# x , y , theta
position = {'x': 5, 'y': 5, 'theta': 5,
            'xd': 0, 'yd': 0, 'thetaD': 0}

# range sensor value
dl = None
df = None
dr = None

ed = None

a = None

# ro
p = 0

motor_status = 'stop'


def range_updater():
    global dl, df, dr
    print('range Sensor thread is running')

    # init range sensors

    l_range_sensors = UltraSonicSensors(pi, l_range_sensors_pins)
    f_range_sensors = UltraSonicSensors(pi, f_range_sensors_pins)
    r_range_sensors = UltraSonicSensors(pi, r_range_sensors_pins)

    while status == run:
        try:
            dl = l_range_sensors.update()
            df = f_range_sensors.update()
            dr = r_range_sensors.update()
            time.sleep(0.1)
        except Exception as e:
            print('range Sensor thread is stopped')
            print(e)
            l_range_sensors.cancel()
            f_range_sensors.cancel()
            r_range_sensors.cancel()
            pi.stop()


def position_updater():
    global position, p, ed, alpha
    while status == run:
        try:
            acceleration, gyro, temp = gyro_sensor.get_all_data()
            position['x'] += acceleration['x']
            position['y'] += acceleration['y']
            position['thetaD'] = gyro['z']

            # ro at i+1
            pi_current = numpy.sqrt(
                (position['xd'] - position['x']) ** 2 +
                (position['yd'] - position['y']) ** 2
            )
            alpha = numpy.arctan(
                (position['yd'] - position['y']) /
                (position['xd'] - position['x'])) - position['theta']
            ed = pi_current - p
            p = pi_current
            time.sleep(0.1)
        except Exception as e:
            print(e)
    return


def reverse(m_speed=None):
    global motor_controller, motor_status
    try:
        # if motor_status != 'backward':
        #     time.sleep(0.1)
        motor_status = 'backward'
        motor_controller.move_backward(back_speed=m_speed)
    # setLEDs(1, 0, 0, 1)
    # print('straight')
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)


def forwards(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'forward'
        motor_controller.move_forward(forward_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)


def turnright(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'right'
        motor_controller.move_right(right_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)


def turnleft(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'left'
        motor_controller.move_left(left_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)


def stopall(force=False):
    global motor_controller, motor_status
    try:
        if force:
            motor_controller.stopall()
        else:
            motor_controller.move_left(left_speed=0)
        motor_status = 'stop'

    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)


# Helper Function
def map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def success():
    current_pos = numpy.array((position['x'], position['y'], position['theta']))
    target_pos = numpy.array((position['xd'], position['yd'], position['thetaD']))
    return numpy.linalg.norm(target_pos - current_pos) > goal_threshold


def auto_movement():
    global status
    fuzzy_system = FuzzySystem()

    while status == run:
        if success():
            status = STOP
        u, w = fuzzy_system.run(dl, df, dr, a, p, ed)
        # linear velocity
        if u > 0:
            forward = map(u, 0, 2, 0, 100)
            forwards(forward)
            time.sleep(0.3)

        # angular velocity
        if w != 0:
            lr_speed = map(w, -4, 4, -100, 100)
            if lr_speed > 0:
                turnright(lr_speed)
            else:
                turnleft(lr_speed)
                time.sleep(0.3)


def print_status(area, fb_speed, is_left, is_right, lr_speed):
    print('******************************')
    print('lr speed is :{} {} {}'.format(lr_speed, is_right, is_left))
    print('x is :{}'.format(area))
    print('******************************')
    print('******************************')
    print('fb speed is :{} {} {}'.format(fb_speed, is_right, is_left))
    print('area is :{}'.format(area))
    print('******************************')


if __name__ == '__main__':
    try:

        range_sensor_thread = threading.Thread(target=range_updater)
        auto_movement_thread = threading.Thread(target=auto_movement)
        position_updater_thread = threading.Thread(target=position_updater)
        range_sensor_thread.start()
        auto_movement_thread.start()
        position_updater_thread.start()

        #  Join Thread to Stop together
        range_sensor_thread.join()
        position_updater_thread.join()
        auto_movement_thread.join()

    finally:
        # Force STOP MOTORS
        stopall(force=True)
        GPIO.cleanup()
