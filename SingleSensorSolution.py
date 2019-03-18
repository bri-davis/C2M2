#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep, perf_counter
import csv
import os.path
import array

robot_speed=-250  #duty_cycle
c=1/100 #1/300forsp=300  #-.68 #4/(.087*.087*-0.213455*3636.364) #P Control constant
a=0 #1/5000#-1/5000#-1/1200  #I Control constant
µ=1  #I's calculation constant

max_motor_speed=100 #duty_cycle
max_steering_angle=50 
wheel_circumference=.29 #meters = 0.174 * 20 / 12 (wheel circ* gear ratio)
degrees_in_rotation=360
target_value=10 #(84-4)/2

def set_u(color_value, current_I):
    steering_angle=c*(color_value-target_value) + a*(current_I-target_value)
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
    modified_motor_speed = min(infrared_sensor_value/60, 1)*motor_duty_cycle
    return modified_motor_speed

def set_p(time_diff):
    p = min(µ*time_diff, 1)
    return p

def set_I(p, previous_I, S):
    current_I = (1-p)*previous_I + p*S
    return current_I

def main():
    hash = 101 * [0]
    #Connects the sensors
    touch_sensor = TouchSensor();    assert touch_sensor.connected, "Connect a touch sensor to any port"   
    color_sensor=ColorSensor(); assert color_sensor.connected, "Connect a color sensor to any sensor port."
    color_sensor.mode='COL-REFLECT'
    #infrared_sensor=InfraredSensor(); assert infrared_sensor.connected, "Connect an infrared sensor to any sensor port."
    #Leds.set_color(Leds.LEFT,  Leds.ORANGE)
    #Leds.set_color(Leds.RIGHT,  Leds.ORANGE)
    #Connects the motors 
    left_motor=LargeMotor('outA'); assert left_motor.connected, "Connect the left motor to port A."
    right_motor=LargeMotor('outD'); assert right_motor.connected, "Connect the right motor to port D."
    ind = 0
    readfilepath ='Intensity2Distance.csv'
    with open(readfilepath, newline="") as f:
        csvreader = csv.reader(f)
        #csvreader.readline()
        next(f)   
        for row in csvreader:
            hash[int(row[2])] = float(row[7])
    	    #i = int(row[0]) # first column of the row
            #print (hash[int(row[2])])
            #ind=ind+1
    #Creates data file
    filepath='DATA/data.csv'
    count=0
    while os.path.isfile(filepath):
        filepath='DATA/data'+str(count)+'.csv'
        count=count + 1

    with open(filepath, 'a+', newline ='') as f:
        filewriter=csv.writer(f)
        filewriter.writerow(["Time Elapsed (s)", "dt (s)", "Distance from Center Line (m)", "I Control", "W Control", "Steering Angle", "LM DutyCycle(%)", "RM DutyCycle(%)", "µ=" + str(µ), "Ω=0", "c=" + str(c), "a=" + str(a)])

        #Initialize variables
        StartTime=perf_counter()
        previous_time=StartTime
        S=target_value
        I=target_value
        w=0
        initial_right_motor_position=right_motor.position
        initial_left_motor_position=left_motor.position
        steering_angle=0
        previous_values = [color_sensor.value()]*8
        past_times = [StartTime]*8              
        prev_index = 0
        count = 0

        while not touch_sensor.value():

            # Collect Timestep and Color Value
            current_time=perf_counter()
            dt=current_time-previous_time
            S=color_sensor.value()

            #D Control with 8 readings in the past
            #prev_index = count%8
            #dS8 = S-prev_values[prev_index]
            #dt8 = current_time - prev_times[prev_index]
            #D=dS8/dt8

            # finding u, uses P & I Control
            p=set_p(dt)
            I=set_I(p, I, S)
            u=set_u(S, I)
           
            steering_angle=limit_steering_angle(u)
            #ir_sensor_value=infrared_sensor.value()
            ir_sensor_value=100
            
            if steering_angle<0:
                left_motor_duty_cycle=modify_slower_motor_speed(steering_angle, robot_speed)
                right_motor_duty_cycle=modify_faster_motor_speed(steering_angle, robot_speed)
                
                left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)
                right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
                #Leds.set_color(Leds.LEFT,  Leds.RED)
                #Leds.set_color(Leds.RIGHT,  Leds.GREEN)
            elif steering_angle>0:
                left_motor_duty_cycle=modify_faster_motor_speed(steering_angle, robot_speed)
                right_motor_duty_cycle=modify_slower_motor_speed(steering_angle, robot_speed)
                right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
                left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)
                #Leds.set_color(Leds.LEFT,  Leds.GREEN)
                #Leds.set_color(Leds.RIGHT,  Leds.RED
            else:
                left_motor_duty_cycle=robot_speed
                right_motor_duty_cycle=robot_speed
                right_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, right_motor_duty_cycle)
                left_motor_duty_cycle=adjust_for_vehicle_separation(ir_sensor_value, left_motor_duty_cycle)
                #Leds.set_color(Leds.LEFT,  Leds.GREEN)
                #Leds.set_color(Leds.RIGHT,  Leds.RED
            #print(str((left_motor_duty_cycle+right_motor_duty_cycle)/2))
            left_motor.run_forever(speed_sp=left_motor_duty_cycle)
            right_motor.run_forever(speed_sp=right_motor_duty_cycle)
            filewriter.writerow([current_time - StartTime, dt, hash[S], I, w, steering_angle, left_motor_duty_cycle, right_motor_duty_cycle])
            previous_time=current_time
            count = count + 1
    Leds.set_color(Leds.LEFT,  Leds.GREEN)
    Leds.set_color(Leds.RIGHT,  Leds.GREEN)
    left_motor.stop(stop_action='brake')
    right_motor.stop(stop_action='brake')
    TotalTravelTime=perf_counter()-StartTime
    total_distance=((right_motor.position - initial_right_motor_position) + (left_motor.position - initial_left_motor_position))*wheel_circumference/(2*degrees_in_rotation)
    print('The total travel time for the robot was: ' + str(TotalTravelTime) + ' seconds')
    print('The average speed of the robot was: ' + str(total_distance/TotalTravelTime) + ' meters per second')
    #print(('The final value of w is: ' + str(previous_w)))
if __name__== "__main__":
    main()
