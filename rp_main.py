import argparse
import os
import socket
import threading
import time
import traceback
from math import sqrt, atan2, degrees, radians

import RPi.GPIO as GPIO
import numpy
# start  pigpiod service
# os.system('sudo pigpiod')
import pigpio

from misc.connection_helper import ConnectionHelper
from misc.motor_controller import QuadMotorController
from misc.range_sensor import UltraSonicSensors

# init controllers
motor_controller = QuadMotorController()
# fuzzy_system = FuzzySystem()
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

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

method = "simple"

run = 'Running'
STOP = 'Stopped'
MANUAL = 'Manual'

status = run
# Simulation Timer
sim_time = 60.0

goal_threshold = 1
# position
# x , y , theta
position = {'x': 0, 'y': 0, 'theta': 0,
            'xd': 0, 'yd': 2, 'thetaD': 0}
loss = numpy.inf
# range sensor value
dl = 2.2
df = 2.2
dr = 2.2
angle = 0
u, w = 10, 0
# angle between the robot heading and the vector connecting the robot center with the target,
# alpha in [-pi, +pi]
alpha = atan2(position['yd'] - position['y'], position['xd'] - position['x']) - position['theta']

p = sqrt(
    pow((position['xd'] - position['x']), 2) +
    pow(position['yd'] - position['y'], 2))

ed = p
motor_status = STOP


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
            dl = round(dl / 50, 2)

            df = f_range_sensors.update()
            df = round(df / 50, 2)

            dr = r_range_sensors.update()
            dr = round(dr / 50, 2)

            time.sleep(0.2)
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
    if method == "moo":
        global loss, goal_threshold, position
        current_pos = numpy.array((position['x'], position['y'], position['theta']))
        target_pos = numpy.array((position['xd'], position['yd'], position['thetaD']))
        loss = numpy.linalg.norm(target_pos - current_pos)
        is_reached = loss < goal_threshold
        return is_reached
    return False


def update_data():
    global socket, dl, df, dr, alpha, p, ed, u, w, angle
    alpha = round(alpha, 2)
    p = round(p, 2)
    ed = round(ed, 2)

    # m_dl = max(min(dl, 4), 0)
    # m_df = max(min(df, 4), 0)
    # m_dr = max(min(dr, 4), 0)
    m_dl = 4
    m_df = 4
    m_dr = 4
    alpha = max(min(alpha, 4), -4)
    p = max(min(p, 20), 0)
    ed = max(min(ed, 1), -1)
    print(f"dl : {m_dl}  df : {m_df}  dr : {m_dr}  alpha : {alpha}  p : {p}  ed : {ed}")

    message = {
        "dl": m_dl,
        "df": m_df,
        "dr": m_dr,
        "alpha": alpha,
        "p": p,
        "ed": ed,
        # map it to [0, 100]
        "velocity": u
    }

    ConnectionHelper.send_json(socket, message)
    result = ConnectionHelper.receive_json(socket)

    print("Got >>", result)

    if "u" in result and "w" in result:
        u = result["u"]
        w = result["w"]
    else:
        u = result["velocity"]
        angle = result["angle"]


u = 0
w = 0

fb_speed = 0
lr_speed = 0


def auto_movement():
    global position, dl, df, dr, alpha, p, ed, alpha, status, u, w, fb_speed, lr_speed, angle
    print('auto movement thread is running')
    move_time = 0.5
    goal_reached = success()
    while status == run and not goal_reached:
        try:
            fb_speed = 0
            # communicate with server
            update_data()
            if method == "moo":
                if w is not None:
                    # lr_speed = int(map(w, -5, 5, -100, 100))
                    lr_speed = degrees(w) / 300
                    print('LR {} '.format(lr_speed))
                    if lr_speed > 0:
                        turnleft(100)
                    elif lr_speed < 0:
                        turnright(100)
                    time.sleep(lr_speed)
                    stopall()
                    # angular velocity
                    # degree = lr_speed * move_time / 360

                    position['theta'] += w  # to Radians
                    position['theta'] = position['theta'] % radians(360)
                    lr_speed = 0
                    time.sleep(0.2)

                if u is not None:
                    fb_speed = int(map(u, 0, 1, 0, 100))
                    print('forward {} '.format(fb_speed))
                    forwards(fb_speed)
                    time.sleep(move_time)
                    stopall()

                    x, y = pol2cart(u * move_time, position['theta'])
                    position['x'] += x
                    position['y'] += y
            else:
                # simple
                x, y = pol2cart(u * move_time, angle)
                position['x'] += x
                position['y'] += y

            # Distance from the center of the robot to the target
            p_current = sqrt(
                pow((position['xd'] - position['x']), 2) +
                pow(position['yd'] - position['y'], 2))

            # angle between the robot heading and the vector connecting the robot center with the target,
            # alpha in [-pi, +pi]
            alpha = atan2(position['yd'] - position['y'], position['xd'] - position['x']) - position['theta']

            ed = p_current - p
            p = p_current
            # print(p_current)
            # print(alpha)
            # print_status(position, fb_speed, lr_speed, dl, df, dr)
            goal_reached = success()
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            break
    print('Robot Stopped because ' + ('goal reached ' if goal_reached else 'Unknown Reason'))
    time.sleep(0.5)
    status = STOP


def print_status():
    global position, fb_speed, lr_speed, dl, df, dr, ed, p
    while status == run:
        os.system('clear')
        print('******************************')
        print('lr speed is : {}\nfb speed is : {} '.format(fb_speed, lr_speed))
        print('******************************')
        print('alpha is : {} \ned is : {}\np is : {} '.format(alpha, ed, p))
        print('******************************')
        print('current Position is :{},{},{}'.format(position['x'], position['y'], degrees(position['theta'])))
        print('******************************')
        print('Destination Position is :{},{},{}'.format(position['xd'], position['yd'], position['thetaD']))
        print('******************************')
        print('Distance L:{} F:{} R:{}'.format(dl, df, dr))
        print('******************************')
        print('Loss is :{}'.format(loss))
        if loss < goal_threshold:
            print('***********************GOAL***REACHED***************************************')
            print('***********************GOAL***REACHED***************************************')
            print('***********************GOAL***REACHED***************************************')
            print('***********************GOAL***REACHED***************************************')
            print('***********************GOAL***REACHED***************************************')
        time.sleep(0.3)


def simulation_timer():
    global status, sim_time
    print('simulation timer has started')
    if sim_time != -1:
        end = time.time() + sim_time
        while time.time() < end and status == run:
            time.sleep(1)
        status = STOP
        time.sleep(2)
    print('simulation timer has stopped')


if __name__ == '__main__':
    try:

        parser = argparse.ArgumentParser()
        parser.add_argument('--host', type=str,
                            default='192.168.1.4')
        parser.add_argument('--port', type=int,
                            default=8888)

        arguments = parser.parse_args()

        try:
            socket.connect((arguments.host, arguments.port))
            print('connected to server ' + arguments.host + ':' + str(arguments.port))
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            status = STOP

        range_sensor_thread = threading.Thread(target=range_updater)
        # fuzzy_thread = threading.Thread(target=do_fuzzy)
        auto_movement_thread = threading.Thread(target=auto_movement)
        simulation_timer_thread = threading.Thread(target=simulation_timer)
        print_thread = threading.Thread(target=print_status)
        # movement_thread = threading.Thread(target=movement)

        range_sensor_thread.start()
        time.sleep(1)
        # movement_thread.start()
        # fuzzy_thread.start()
        print_thread.start()
        simulation_timer_thread.start()
        auto_movement_thread.start()

        #  Join Threads to Stop together
        # movement_thread.join()
        range_sensor_thread.join()
        # fuzzy_thread.join()
        auto_movement_thread.join()
        simulation_timer_thread.join()
        print_thread.join()
    finally:
        # Force STOP MOTORS
        stopall(force=True)
        GPIO.cleanup()
