#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep, perf_counter
import csv
import os.path
import array
import math

robot_speed=-230  #duty_cycle
c=-1/320 #-1/350 #1/300forsp=300  #-.68 #4/(.087*.087*-0.213455*3636.364) #P Control constant
a=0#-1/4000#1/3000 #1/6000 #-1/5000#-1/1200  #I Control constant
b=-5/10000#1/200
µ=1  #I's calculation constant

max_motor_speed=100 #duty_cycle
max_steering_angle=50
wheel_circumference=.29 #meters = 0.174 * 20 / 12 (wheel circ* gear ratio)
degrees_in_rotation=360
target_value=43 #(84-4)/2


def set_u(current_P, current_I, current_D):
    steering_angle=c*(current_P) + a*(current_I) + b*(current_D)
    return steering_angle

def limit_steering_angle(steering_angle):
    steering_angle = min(max(steering_angle, (-1)), 1)
    return steering_angle
def modify_faster_motor_speed(steering_angle, motor_speed):
    modified_motor_speed = abs(steering_angle)*(1/abs(steering_angle) + 1)*motor_speed
    return modified_motor_speed

def modify_slower_motor_speed(steering_angle, motor_speed):
    modified_motor_speed = abs(steering_angle)*(1/abs(steering_angle) - 1)*motor_speed
    return modified_motor_speed

def adjust_for_vehicle_separation(infrared_sensor_value, motor_duty_cycle):
    modified_motor_speed = min(max((infrared_sensor_value-10)/60, 0), 1)*motor_duty_cycle
    return modified_motor_speed

def set_p(time_diff):
    p = min(µ*time_diff, 1)
    return p

def set_I(p, previous_I, Sdiff):
    current_I = (1-p)*previous_I + p*Sdiff
    return current_I

hash = 101 * [0] #array for color intensity -> distance from center of track (m)
readfilepath ='EditedIntensityMap.csv' #File that has data for intensity -> distance
with open(readfilepath, newline="") as f:
    csvreader = csv.reader(f)
    for row in csvreader:
        hash[int(row[0])] = float(float(row[1]))
    for i in range(0, 100):
        print("at"+str(i)+":"+str(hash[i]))
        
#Connect the sensors
touch_sensor = TouchSensor(); assert touch_sensor.connected, "Connect a touch sensor to port 2 or 3."
color_sensor_right=ColorSensor('in1'); assert color_sensor_right.connected, "Connect the left color sensor to port 1."
color_sensor_right.mode='COL-REFLECT'
color_sensor_left = ColorSensor('in4'); assert color_sensor_left.connected, "Connect the right color sensor to port 4."
color_sensor_left.mode='COL-REFLECT'
infrared_sensor=InfraredSensor(); assert infrared_sensor.connected, "Connect an infrared sensor to port 2 or 3."

#Connect the motors
left_motor=LargeMotor('outA'); assert left_motor.connected, "Connect the left motor to port A."
right_motor=LargeMotor('outD'); assert right_motor.connected, "Connect the right motor to port D."

#Initialize variables
start_time=perf_counter()
previous_time=start_time
RS=target_value
LS=target_value
I=target_value
initial_right_motor_position=right_motor.position
initial_left_motor_position=left_motor.position
steering_angle=0
previous_values = [color_sensor_right.value()]*8
previous_times = [start_time]*8
prev_index = 0
count = 0
std=0
std_sum=0
std_iter=0
total_decisions = 0
index=0
prev_8th_S=0
while not touch_sensor.value():

    #P Control
    #input: nothing
    #output: position S (m), time t (s)
    current_time=perf_counter()
    RS=color_sensor_right.value()
    LS=color_sensor_left.value()
    S=LS-RS

    # D Control
    #input: positon S (m), time t (s)
    #output: speed dS/dt (m/s)
    index=count%8
    prev_8th_S=previous_values[index]
    previous_values[index]=S
    prev_8th_time=previous_times[index]
    previous_times[index]=current_time
    D=(S-prev_8th_S)/(current_time-prev_8th_time)
    print(current_time-prev_8th_time)
    # I Control
    #input: positon S (m), time t (s)
    #output: integral of position S with respect to time t (ms)
    dt=current_time-previous_time
    p=set_p(dt)
    S_diff=S-previous_values[index]
    I=set_I(p, I, S_diff)#running average

    # Steering Adjustment
    #input: positon S (m), time t (s), speed dS/dt (m/s), integral of position S with respect to time t (ms)
    #output: steering adjustment steering_angle
    u=set_u(S, I, D)
    steering_angle=limit_steering_angle(u)

    #Vehicle Vision
    ir_sensor_value=infrared_sensor.value()
    #ir_sensor_value=100

    if steering_angle<0:
        left_motor_duty_cycle=modify_slower_motor_speed(steering_angle, robot_speed)
        right_motor_duty_cycle=modify_faster_motor_speed(steering_angle, robot_speed)

        left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)
        right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
    elif steering_angle>0:
        left_motor_duty_cycle=modify_faster_motor_speed(steering_angle, robot_speed)
        right_motor_duty_cycle=modify_slower_motor_speed(steering_angle, robot_speed)
        
        right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
        left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)

    else: #steering_angle==0
        left_motor_duty_cycle=robot_speed
        right_motor_duty_cycle=robot_speed
        
        right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
        left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)
    left_motor.run_forever(speed_sp=left_motor_duty_cycle)
    right_motor.run_forever(speed_sp=right_motor_duty_cycle)
    previous_time=current_time
    count+=1
Leds.set_color(Leds.LEFT,  Leds.GREEN)
Leds.set_color(Leds.RIGHT,  Leds.GREEN)
left_motor.stop(stop_action='brake')
right_motor.stop(stop_action='brake')
TotalTravelTime=perf_counter()-start_time
total_distance=((right_motor.position - initial_right_motor_position) + (left_motor.position - initial_left_motor_position))*wheel_circumference/(2*degrees_in_rotation)
print('The total travel time for the robot was: ' + str(TotalTravelTime) + ' seconds')
#print('The average speed of the robot was: ' + str(total_distance/TotalTravelTime) + ' meters per second')
#std_sum = std_sum/count
#std=std_sum
#print(('The final value of stditer is: ' + str(std_)))
#print(('The final value of std is: ' + str(std_sum)))
