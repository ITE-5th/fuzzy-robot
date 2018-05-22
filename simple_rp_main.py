import argparse
import os
import socket
import threading
import time
import traceback

import RPi.GPIO as GPIO
# start  pigpiod service
# os.system('sudo pigpiod')
import pigpio

from fuzzy_system.simple_fuzzy_system import SimpleFuzzySystem
from misc.connection_helper import ConnectionHelper
from misc.motor_controller import QuadMotorController
from misc.range_sensor import UltraSonicSensors

# init controllers
motor_controller = QuadMotorController()
fuzzy_system = None
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
x_d, y_d, theta_d = 2, 0, 0

# range sensor value
dl = 2.2
df = 2.2
dr = 2.2
angle = 0
u, w = 10, 0

motor_status = STOP
first_time = True

# calculations precision
degree = 2


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
            dl = round(dl * 0.5, 2)

            df = f_range_sensors.update()
            df = round(df * 0.5, 2)

            dr = r_range_sensors.update()
            dr = round(dr * 0.5, 2)

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


def do_fuzzy():
    global dl, df, dr, status, u, angle
    print('Fuzzy system is activated')
    while status == run:
        try:
            front, left, right, velocity = df, dl, dr, u
            front = min(max(front, 0), 70)
            left = min(max(left, 0), 70)
            right = min(max(right, 0), 70)
            print(f'front : {front},left:{left},right:{right}')
            values = {
                "front": front,
                "left": left,
                "right": right,
                "velocity": velocity
            }
            u, angle = fuzzy_system.run(values)

        except Exception as e:
            print(e)
            print(traceback.format_exc())
            status = STOP
            break
    print('Fuzzy system is Deactivated')


def update_data():
    global socket, dl, df, dr, u, angle, first_time
    if first_time:
        message = {
            "method": 'simple'
        }
        ConnectionHelper.send_json(socket, message)
        ConnectionHelper.receive_json(socket)
        first_time = False

    # take sensors current value
    current_dl = dl
    current_df = df
    current_dr = dr

    current_dl = round(current_dl, degree)
    current_df = round(current_df, degree)
    current_dr = round(current_dr, degree)

    print(f"dl : {current_dl}  df : {current_df}  dr : {current_dr} velocity:{u}")
    message = {
        "dl": current_dl,
        "df": current_df,
        "dr": current_dr,
        "velocity": u
    }

    ConnectionHelper.send_json(socket, message)
    result = ConnectionHelper.receive_json(socket)

    print("Got >>", result)
    u = result["velocity"]
    angle = result["angle"]


fb_speed = 0
lr_speed = 0


def auto_movement():
    global status, lr_speed
    print('auto movement thread is running')
    prev_u = -1
    while status == run:
        try:
            # communicate with server
            if use_server:
                update_data()
            if angle is not None and angle != 0 and abs(angle) > 25:
                degree_per_second = 375
                lr_speed = round(abs(angle / degree_per_second), 2)
                print('LR {} '.format(lr_speed))
                if angle > 0:
                    turnleft(100)
                elif angle < 0:
                    turnright(100)
                time.sleep(lr_speed)
                stopall()
                continue

            if u is not None and u != 0:
                forwards(u)
                time.sleep(0.3)
                stopall()

        except Exception as e:
            print(e)
            print(traceback.format_exc())
            stopall()
            break
    print('Robot Stopped')
    time.sleep(0.5)
    status = STOP


def print_status():
    while status == run:
        os.system('clear')
        print('******************************')
        print('lr speed is : {}\nfb speed is : {} '.format(angle, u))
        print('******************************')
        print('Distance L:{} F:{} R:{}'.format(dl, df, dr))
        print('******************************')
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


use_server = True

if __name__ == '__main__':

    try:

        parser = argparse.ArgumentParser()
        parser.add_argument('--host', type=str,
                            default='')
        parser.add_argument('--port', type=int,
                            default=8888)

        arguments = parser.parse_args()
        use_server = arguments.host != ''
        if use_server:
            try:
                socket.connect((arguments.host, arguments.port))
                print('connected to server ' + arguments.host + ':' + str(arguments.port))
            except Exception as e:
                print(e)
                print(traceback.format_exc())
                status = STOP
        else:
            print('initializing Fuzzy System')
            fuzzy_system = SimpleFuzzySystem()
            print('Fuzzy System initialized')

        fuzzy_thread = threading.Thread(target=do_fuzzy)
        range_sensor_thread = threading.Thread(target=range_updater)
        auto_movement_thread = threading.Thread(target=auto_movement)
        simulation_timer_thread = threading.Thread(target=simulation_timer)
        print_thread = threading.Thread(target=print_status)

        range_sensor_thread.start()
        time.sleep(1)
        print_thread.start()
        simulation_timer_thread.start()
        auto_movement_thread.start()
        if not use_server:
            fuzzy_thread.start()
            fuzzy_thread.join()

        #  Join Threads to Stop together
        # movement_thread.join()
        range_sensor_thread.join()
        auto_movement_thread.join()
        simulation_timer_thread.join()
        print_thread.join()
    finally:
        # Force STOP MOTORS
        stopall(force=True)
        GPIO.cleanup()
