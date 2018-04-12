import threading
import time

import RPi.GPIO as GPIO
import pigpio

from fuzzy_controller.fuzzy_system import FuzzySystem
from misc.motor_controller import QuadMotorController
from misc.range_sensor import sensor

run = 'Running'
STOP = 'Stopped'
MANUAL = 'Manual'

fb_speed = 0
lr_speed = 0

x = 500
y = 0
area = 7000
x_min = 200
x_max = 800

motor_controller = QuadMotorController()

no_object = "no_object"
motor_status = 'stop'


def range_sensor_updater():
    global range_sensor_value
    print('range Sensor thread is running')
    pi = pigpio.pi()
    sonar = sensor(pi, trigger=23, echo=24)
    while True:
        try:
            sonar.trigger()
            time.sleep(0.1)
            cms, new = sonar.get_centimetres()
            range_sensor_value = 100
            # if new:
            #     range_sensor_value = cms
            #     # print('range : {}'.format(range_sensor_value))
        except Exception as e:
            print('range Sensor thread is stopped')
            print(e)
            sonar.cancel()
            pi.stop()


def reverse(m_speed=None):
    global motor_controller, motor_status
    try:
        # if motor_status != 'backward':
        #     time.sleep(0.1)
        motor_status = 'backward'
        motor_controller.move_backward(back_speed=m_speed)
    # setLEDs(1, 0, 0, 1)
    # print('straight')
    except:
        motor_controller = QuadMotorController()


def forwards(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'forward'
        if range_sensor_value > 30:
            motor_controller.move_forward(forward_speed=m_speed)
        else:
            print('RANGE SENSOR : {} cm'.format(range_sensor_value))
    except:
        motor_controller = QuadMotorController()


def turnright(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'right'
        motor_controller.move_right(right_speed=m_speed)
    except:
        motor_controller = QuadMotorController()


def turnleft(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_status = 'left'
        motor_controller.move_left(left_speed=m_speed)
    except:
        motor_controller = QuadMotorController()


def stopall(force=False):
    global motor_controller, motor_status
    try:
        if force:
            motor_controller.stopall()
        else:
            motor_controller.move_left(left_speed=0)
        motor_status = 'stop'
    except:
        motor_controller = QuadMotorController()


# Helper Function
def map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def auto_movement():
    global status, x_min, x_max, x, y, fb_speed, lr_speed, area
    last_turn = 'L'
    no_object_loops = 0
    fuzzy_system = FuzzySystem()
    while True:
        # todo
        # u, w = fuzzy_system.run()
        if status == run:
            no_object_loops = 0

            # Calculate PID updates

            fb_update = fb_pid.update(area)
            fb_speed = percentage(fb_update, fb_pid.target)
            #
            is_forward = fb_speed > 20
            is_backward = fb_speed < -20

            fb_speed = max(0, min(100, abs(int(fb_speed))))
            fb_speed = int(map(fb_speed, 0, 100, 0, 100))

            lr_update = lr_pid.update(x)
            lr_speed = percentage(lr_update, lr_pid.target)
            #
            is_left = lr_speed > 30
            is_right = lr_speed < -30
            #
            lr_speed = max(0, min(100, abs(int(lr_speed))))
            lr_speed = int(map(lr_speed, 0, 100, 0, 75))

            print_status(area, fb_speed, is_left, is_right, lr_speed)

            if is_left:
                turnleft(m_speed=lr_speed)
                x += 100 * (lr_speed / 100)
                last_turn = 'L'
            elif is_right:
                turnright(m_speed=lr_speed)
                x -= 100 * (lr_speed / 100)
                last_turn = 'R'
            elif is_forward:
                forwards(m_speed=fb_speed)
                area += 400 * (fb_speed / 100)
            elif is_backward:
                reverse(m_speed=fb_speed)
                area -= 400 * (fb_speed / 100)

            if no_obj_flag and not is_left and not is_right and not is_forward and not is_backward:
                status = no_object
            time.sleep(0.05)
            stopall()
        elif status == no_object:
            print('********** No Object **********')
            if no_object_loops < 10:
                no_object_loops += 1
                if last_turn == 'R':
                    turnright(m_speed=50)
                else:
                    turnleft(m_speed=50)
                time.sleep(0.1)
                stopall()
                time.sleep(0.5)
        else:
            time.sleep(1)


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

        range_sensor_thread = threading.Thread(target=range_sensor_updater)
        auto_movement_thread = threading.Thread(target=auto_movement)
        # range_sensor_thread.start()
        auto_movement_thread.start()

        #  Join Thread to Stop together
        # range_sensor_thread.join()
        auto_movement_thread.join()

    finally:
        # Force STOP MOTORS
        stopall(force=True)
        GPIO.cleanup()
