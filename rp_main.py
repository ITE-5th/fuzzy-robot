import threading
import time
import traceback
from math import sqrt

import RPi.GPIO as GPIO
import numpy
import pigpio

from fuzzy_controller.fuzzy_system import FuzzySystem
from misc.motor_controller import QuadMotorController
from misc.range_sensor import UltraSonicSensors

# init controllers
motor_controller = QuadMotorController()
fuzzy_system = FuzzySystem()
pi = pigpio.pi()

r_range_sensors_pins = {
    # R
    (23, 24),
    (23, 22),
    (23, 27),
}
f_range_sensors_pins = {
    # F
    (23, 17),
    (23, 4),
}

l_range_sensors_pins = {
    # L
    (23, 18),
    (23, 25),
    (23, 12),

}

run = 'Running'
STOP = 'Stopped'
MANUAL = 'Manual'

status = run
# Simulation Timer
sim_time = 30.0

goal_threshold = 1
# position
# x , y , theta
position = {'x': 0, 'y': 0, 'theta': 0,
            'xd': 100, 'yd': 100, 'thetaD': 100}
loss = numpy.inf
# range sensor value
dl = 6.2
df = 4.2
dr = 1.2

alpha = 0
ed = 1

# ro
p = 0

motor_status = 'stop'


def range_updater():
    global dl, df, dr, status
    print('range Sensor thread is running')

    # init range sensors

    l_range_sensors = UltraSonicSensors(pi, l_range_sensors_pins)
    f_range_sensors = UltraSonicSensors(pi, f_range_sensors_pins)
    r_range_sensors = UltraSonicSensors(pi, r_range_sensors_pins)

    while status == run:
        try:

            dl = l_range_sensors.update()
            dl = round(map(dl, 0, 200, 0, 4), 2)

            df = f_range_sensors.update()
            df = round(map(df, 0, 200, 0, 4), 2)

            dr = r_range_sensors.update()
            dr = round(map(dr, 0, 200, 0, 4), 2)

            time.sleep(0.1)
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            l_range_sensors.cancel()
            f_range_sensors.cancel()
            r_range_sensors.cancel()
            pi.stop()
            status = STOP
            break
    print('range Sensor thread is stopped')


def reverse(m_speed=None):
    global motor_controller, motor_status
    try:
        motor_controller.move_backward(back_speed=m_speed)
    # setLEDs(1, 0, 0, 1)
    # print('straight')
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)
        print(traceback.format_exc())


def forwards(m_speed=None):
    global motor_controller, motor_status
    try:
        print('forward')
        motor_controller.move_forward(forward_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)
        print(traceback.format_exc())


def turnright(m_speed=None):
    global motor_controller, motor_status
    try:
        print('right')
        motor_controller.move_right(right_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)
        print(traceback.format_exc())


def turnleft(m_speed=None):
    global motor_controller, motor_status
    try:
        print('left')
        motor_controller.move_left(left_speed=m_speed)
    except Exception as e:
        motor_controller = QuadMotorController()
        print(e)
        print(traceback.format_exc())


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
        print(traceback.format_exc())


# Helper Functions
def map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def pol2cart(rho, phi):
    x = rho * numpy.cos(phi)
    y = rho * numpy.sin(phi)
    return x, y


def success():
    global loss, goal_threshold
    current_pos = numpy.array((position['x'], position['y'], position['theta']))
    target_pos = numpy.array((position['xd'], position['yd'], position['thetaD']))
    loss = numpy.linalg.norm(target_pos - current_pos)
    print(goal_threshold)
    return loss < goal_threshold


u = 0
w = 0


def do_fuzzy():
    global dl, df, dr, alpha, p, ed, status, u, w
    print('Fuzzy system is activated')
    while status == run:
        try:
            print('{} {} {} {} {} {}'.format(dl, df, dr, alpha, p, ed))
            print('{} {} {} {} {} {}'.format(dl, df, dr, alpha, p, ed))
            print('{} {} {} {} {} {}'.format(dl, df, dr, alpha, p, ed))
            print('{} {} {} {} {} {}'.format(dl, df, dr, alpha, p, ed))
            u, w = fuzzy_system.run(dl, df, dr, 1, 1, 1)
            print('step')
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            status = STOP
            break
    print('Fuzzy system is Deactivated')


def auto_movement():
    global position, dl, df, dr, alpha, p, ed, alpha, status, u, w
    print('auto movement thread is running')

    while status == run and not success():
        try:
            fb_speed = 0

            if u is not None:
                fb_speed = int(map(u, 0, 2, 0, 100))
                forwards(fb_speed)
                time.sleep(0.3)
                stopall()
                # angular velocity
            lr_speed = 0
            if w is not None:
                lr_speed = int(map(w, -5, 5, -100, 100))
                if lr_speed > 0:
                    turnright(lr_speed)
                else:
                    turnleft(abs(lr_speed))
                time.sleep(0.3)
                stopall()
                position['thetaD'] += w

            # ro at i+1

            x, y = pol2cart(u, position['theta'])
            position['x'] += x
            position['y'] += y

            p_current = sqrt(
                pow((position['xd'] - position['x']), 2) +
                pow(position['yd'] - position['y'], 2))
            alpha = numpy.arctan(
                (position['yd'] - position['y']) /
                (position['xd'] - position['x'])) - position['theta']
            ed = p_current - p
            p = p_current
            print(p_current)
            print(alpha)
            print_status(position, fb_speed, lr_speed, dl, df, dr)
        except Exception as e:
            print(e)
            print(traceback.format_exc())

            status = STOP
            break
    print('auto movement thread is stopped')


def print_status(pos, fb_speed, lr_speed, dl, df, dr):
    # os.system('clear')
    print('******************************')
    print('lr speed is : {}\nfb speed is : {} '.format(fb_speed, lr_speed))
    print('******************************')
    print('current Position is :{},{},{}'.format(pos['x'], pos['y'], pos['theta']))
    print('******************************')
    print('Destination Position is :{},{},{}'.format(pos['xd'], pos['yd'], pos['thetaD']))
    print('******************************')
    print('Distance L:{} F:{} R:{}'.format(dl, df, dr))
    print('******************************')
    print('Loss is :{}'.format(loss))
    print('******************************')


def simulation_timer():
    global status, sim_time
    print('simulation timer has started')
    if sim_time != -1:
        end = time.time() + sim_time
        while time.time() < end:
            time.sleep(1)
        status = STOP
        time.sleep(2)
        print('simulation stopped because timeout')
    print('simulation timer has stopped')


if __name__ == '__main__':
    try:

        range_sensor_thread = threading.Thread(target=range_updater)
        fuzzy_thread = threading.Thread(target=do_fuzzy)
        # auto_movement_thread = threading.Thread(target=auto_movement)
        simulation_timer_thread = threading.Thread(target=simulation_timer)

        range_sensor_thread.start()

        fuzzy_thread.start()
        simulation_timer_thread.start()
        # auto_movement_thread.start()

        #  Join Thread to Stop together
        # range_sensor_thread.join()
        fuzzy_thread.join()
        # auto_movement_thread.join()
        simulation_timer_thread.join()

    finally:
        # Force STOP MOTORS
        stopall(force=True)
        GPIO.cleanup()
