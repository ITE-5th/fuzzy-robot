import argparse
import os
import socket
import threading
import time
import traceback
from math import atan2, degrees, radians, hypot

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

run = 'Running'
STOP = 'Stopped'
MANUAL = 'Manual'

status = run
# Simulation Timer
sim_time = 60.0

goal_threshold = 0.5

# x , y , theta

# robot position and orientation
x, y, theta = 0, 0, 0

# target position and orientation
x_d, y_d, theta_d = 2, 2, 0

dist = numpy.inf
# range sensor value
dl = 2.2
df = 2.2
dr = 2.2

# angle between the robot heading and the vector connecting the robot center with the target,
# alpha in [-pi, +pi]
alpha = atan2(y_d - y, x_d - x) - theta

p = hypot(x_d - x, y_d - y)

ed = p
motor_status = STOP

# calculations precision
degree = 10


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
    global dist, goal_threshold
    dist = hypot(x_d - x, y_d - y)
    is_reached = dist < goal_threshold
    return is_reached


def update_data():
    global socket, dl, df, dr, alpha, p, ed, u, w

    # angle between the robot heading and the vector connecting the robot center with the target,
    # alpha in [-pi, +pi]
    alpha = atan2(y_d - y, x_d - x) - theta
    alpha = atan2(y_d - y, x_d - x) - theta

    # Distance from the center of the robot to the target
    p_current = hypot(x_d - x, y_d - y)

    ed = p_current - p
    p = p_current

    # take sensors current value

    # current_dl = dl
    # current_df = df
    # current_dr = dr

    current_dl = 4
    current_df = 4
    current_dr = 4
    alpha = round(alpha, degree)
    p = round(p, degree)
    ed = round(ed, degree)
    current_dl = round(current_dl, degree)
    current_df = round(current_df, degree)
    current_dr = round(current_dr, degree)

    # ensure that data in fuzzy range
    alpha = max(min(alpha, 4), -4)
    p = max(min(p, 20), 0)
    ed = max(min(ed, 1), -1)
    current_dl = max(min(current_dl, 4), 0)
    current_df = max(min(current_df, 4), 0)
    current_dr = max(min(current_dr, 4), 0)

    print(f"dl : {current_dl}  df : {current_df}  dr : {current_dr}  alpha : {alpha}  p : {p}  ed : {ed}")
    message = {
        "dl": current_dl,
        "df": current_df,
        "dr": current_dr,
        "alpha": alpha,
        "p": p,
        "ed": ed
    }

    ConnectionHelper.send_json(socket, message)
    result = ConnectionHelper.receive_json(socket)

    print("Got >>", result)

    if "u" in result and "w" in result:
        u = result["u"]
        w = result["w"]


u = 0
w = 0

fb_speed = 0
lr_speed = 0


def auto_movement():
    global x, y, theta, x_d, y_d, theta_d, dl, df, dr, p, ed, alpha, status, u, w, fb_speed, lr_speed
    print('auto movement thread is running')
    move_time = 0.5
    goal_reached = success()
    while status == run and not goal_reached:
        try:
            fb_speed = 0
            # communicate with server
            update_data()
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

                theta += w
                theta = theta % radians(360)
                lr_speed = 0
                time.sleep(0.2)

            if u is not None:
                fb_speed = int(map(u, 0, 1, 0, 100))
                print('forward {} '.format(fb_speed))
                forwards(fb_speed)
                time.sleep(move_time)
                stopall()
                x, y = pol2cart(u * move_time, theta)
                x += x
                y += y

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
    while status == run:
        os.system('clear')
        print('******************************')
        print('lr speed is : {}\nfb speed is : {} '.format(fb_speed, lr_speed))
        print('******************************')
        print('alpha is : {} \ned is : {}\np is : {} '.format(alpha, ed, p))
        print('******************************')
        print('current Position is :{},{},{}'.format(x, y, degrees(theta)))
        print('******************************')
        print('Destination Position is :{},{},{}'.format(x_d, y_d, theta_d))
        print('******************************')
        print('Distance L:{} F:{} R:{}'.format(dl, df, dr))
        print('******************************')
        print('dist is :{}'.format(dist))
        if dist < goal_threshold:
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
