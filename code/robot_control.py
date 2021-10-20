from threading import Thread
import numpy as np
import pandas as pd
from LineFollower import LineFollower
from SerialDevice import yesno_confirm


robot = LineFollower()

# sensor thresholds for determining when we're fully over the floor and should try to find the line again
LEFT_THRESHOLD = 200
RIGHT_THRESHOLD = 200

# sensor bounds - the lowest and highest values read during calibration
LEFT_MIN = 70
LEFT_MAX = 432
RIGHT_MIN = 59
RIGHT_MAX = 430

Kp = .1         # proportionality constant for P control - multiplied by error to get a motor speed adjustment
driving = False # 
kill = False    # used to stop the control thread and save the collected data as a .csv

base_speed = 45
max_speed = 70
min_speed = 25


def calibrate():
    '''
    Acquires calibration data.

    Turns the robot clockwise while recording sensor data, keeping track of the lowest and highest values for each sensor.
    Stops and checks if the user wants to finish calibrating after every 50 readings.

    Returns:
        [[left_sensor_min: int, left_sensor_max: int], [right_sensor_min: int, right_sensor_max: int]]
    '''
    readings = [[], []]
    check_length = 0
    robot.left_speed = 25
    robot.right_speed = 25
    robot.left_state = 'FORWARD'
    robot.right_state = 'BACKWARD'

    while True:
        robot.left_speed = 25
        robot.right_speed = 25
        robot.left_state = 'FORWARD'
        robot.right_state = 'BACKWARD'
        readings[0].append(robot.read_sensor('left'))
        readings[1].append(robot.read_sensor('right'))
        if len(readings[1]) - check_length > 50:
            check_length += 50
            robot.left_state = 'RELEASE'
            robot.right_state = 'RELEASE'
            print(f'Left min: {np.amin(readings[0])}')
            print(f'Left max: {np.amax(readings[0])}')
            print(f'Right min: {np.amin(readings[1])}')
            print(f'Right max: {np.amax(readings[1])}')
            if not yesno_confirm('Continue calibration?'):
                return [[np.amin(readings[0]), np.amax(readings[0])], [np.amin(readings[1]), np.amax(readings[1])]]


def drive_robot(initial_bounds=[[LEFT_MIN,LEFT_MAX],[RIGHT_MIN,RIGHT_MAX]]):
    '''
    Robot controller utilizing P control (PID without I or D). Attempts to minimize the difference between the two sensor
    readings (adjusting for their minimum readings) while moving the robot forward in order to keep it centered along the line.
    If the sensors lose sight of the line, the robot will turn in the previous direction it turned until finding it again.

    Args:
        initial_bounds (list, optional): Calibration data (sensor min and max readings) that can be generated with calibration().
            Defaults to [[LEFT_MIN,LEFT_MAX],[RIGHT_MIN,RIGHT_MAX]].
    '''
    # lists for data collection
    left_readings = []
    right_readings = []
    left_speeds = []
    right_speeds = []

    bounds = initial_bounds

    # 0 for left, 1 for right - used for turning the right direction to find the track again if
    # we lose sight of it - defaults to left, but it gets updated pretty much instantly anyways
    last_turn = 0
    
    # kill is set to true when we want to terminate the thread from the console (triggered on
    # KeyboardInterrupt, see end of main())
    while not kill:
        if driving:
            # get sensor readings and record them
            readings = [robot.read_sensor('left'), robot.read_sensor('right')]
            left_readings.append(readings[0])
            right_readings.append(readings[1])

            # update min and max
            if readings[0] < bounds[0][0]:
                bounds[0][0] = readings[0]
            elif readings[0] > bounds[0][1]:
                bounds[0][1] = readings[0]
            if readings[1] < bounds[1][0]:
                bounds [1][0] = readings[1]
            elif readings[1] > bounds[1][1]:
                bounds [1][1] = readings[1]

            # stop wheels if the robot is picked up (reflectance close to 0)
            if any([reading < 5 for reading in readings]):
                robot.left_state = 'RELEASE'
                robot.right_state = 'RELEASE'
            else:   # go forward otherwise
                robot.left_state = 'FORWARD'
                robot.right_state = 'FORWARD'

            # if we lost sight of the track, turn the most recent direction until finding it again
            if readings[0] > LEFT_THRESHOLD and readings[1] > RIGHT_THRESHOLD:
                robot.left_speed = base_speed
                robot.right_speed = base_speed
                if last_turn:
                    robot.left_state = 'FORWARD'
                    robot.right_state = 'BACKWARD'
                    # record data
                    left_speeds.append(base_speed)
                    right_speeds.append(-base_speed)
                else:
                    robot.left_state = 'BACKWARD'
                    robot.right_state = 'FORWARD'
                    # record data
                    left_speeds.append(-base_speed)
                    right_speeds.append(base_speed)
            else:
                # adjust the readings based on the sensor minimums (subtract the minimums from the raw readings)
                # this way we account for differences in the actual hardware i.e. different
                # values for being in the center of the line
                readings = [readings[0]-bounds[0][0], readings[1]-bounds[0][0]]
                
                # we want to minimize the difference between the readings, so our error term is based on this
                error = readings[1] - readings[0]
                # update last_turn
                # positive error -> left turn
                if error > 0:
                    last_turn = 0
                else: # negative error -> right turn
                    last_turn = 1
                
                # calculate motor speeds based on error
                motor_speed = int(error * Kp)
                left_speed = base_speed - motor_speed
                right_speed = base_speed + motor_speed

                # constrain speeds to max and mins
                left_speed = max_speed if left_speed > max_speed else left_speed
                left_speed = min_speed if left_speed < min_speed else left_speed
                right_speed = max_speed if right_speed > max_speed else right_speed
                right_speed = min_speed if right_speed < min_speed else right_speed

                # apply new speeds to motors
                robot.left_speed = left_speed
                robot.right_speed = right_speed
                # record data
                left_speeds.append(left_speed)
                right_speeds.append(right_speed)
        else:
            robot.left_state = 'RELEASE'
            robot.right_state = 'RELEASE'
    # save the recorded data to a csv when the thread is stopped
    pd.DataFrame({'left sensor': left_readings, 'right sensor': right_readings, 'left motor': left_speeds, 'right motor': right_speeds}).to_csv('data.csv')

def main():
    # globals for changing robot behavior in real time
    global driving, Kp, base_speed, kill, min_speed, max_speed

    # start the robot drive thread - this runs at the same time as the input stuff below, letting you modify the
    # robot's behavior while it runs
    # drive_thread = Thread(target=drive_robot, args=(calibrate(),))    # uncomment this line and comment next line to calibrate the sensors before driving
    drive_thread = Thread(target=drive_robot)
    drive_thread.daemon = True
    drive_thread.start()
    try: 
        while True:
            choice = input('0: stop/start robot\n'+
                            'k=[number]: set a new Kp value\n'+
                            's=[number]: set a new base speed value\n'+
                            'min=[number]: set a new minimum speed value\n'+
                            'max=[number]: set a new maximum speed value\n'+
                            'Choose one: ')
            try:    # whole thing wrapped in try/except to catch errors from bad input
                if choice == '0':
                    driving = not driving   # toggle driving state
                elif '=' in choice:
                    new_val = choice.split('=')[1]  # try to grab the new value from the string
                    if choice[0:2] == 'k=':
                        Kp = float(new_val) # update proportion constant
                    elif choice[0:2] == 's=':
                        new_val = int(new_val)
                        if new_val not in range(0,255): # even if the conversion is valid, the speed is still constrained to 0-255
                            raise ValueError
                        base_speed = new_val # update base speed
                    elif choice[0:4] == 'min=':
                        new_val = int(new_val)
                        if new_val not in range(0,255):
                            raise ValueError
                        min_speed = new_val # update minimum speed
                    elif choice[0:4] == 'max=':
                        new_val = int(new_val)
                        if new_val not in range(0,255):
                            raise ValueError
                        max_speed = new_val # update maximum speed
            except ValueError:
                print('invalid value!')
    except KeyboardInterrupt:   # make sure the drive thread stops when the console interface is killed
        kill = True
        drive_thread.join(5)

if __name__ == '__main__':
    main()
