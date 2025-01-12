from machine import Pin, PWM
from time import sleep, ticks_ms, ticks_add, ticks_diff
from MPU_data import MPU6050DATA
from RFClass import RFClass
import os

# From 1/12/2025
# - with a while loop
# - detects angle instead of angular speed
# - with PID
# - DOWNLOAD THE FILE BEFORE UNPLUGGING THE BATTERY
# - add in RF message to include controller adjustments
#     [0] take off/landing, [1:3] pitch, [3:5] roll, [5:7] yaw, [7:9] throttle adjustment (+-0.01 by each click)

# Future: add error value. To be controller - gyro data
# 		  add pid for yaw & kd
# LED indication:
# - end with on: toggled checkRotation()
# - end with off: finished the entire loop

led = Pin(25, Pin.OUT)

mpu = MPU6050DATA(id=0, sda=12, scl=13)
rf = RFClass()

esc_1 = PWM(Pin(15)) # Top Left - weaker one - changed to pin 15, originally pin 14 
esc_2 = PWM(Pin(2)) # bottom left - ccw
esc_3 = PWM(Pin(28)) # Bottom right
esc_4 = PWM(Pin(16)) # Top right - ccw

esc_1.freq(50)
esc_2.freq(50)
esc_3.freq(50)
esc_4.freq(50)

# deletes current file
# if os.path.exists('/angleData.txt'):
#try commenting out os.remove
os.remove('/angleData.txt')
os.remove('/throttleData.txt')
# Open file to log data
angleFile = open('angleData.txt', 'w') # mode(r, a, w, x, t,b)
throttleFile = open('throttleData.txt', 'w')
rcvdFile = open('rcvd.txt','r')
        
period_ms = 20
max_throttle = int ((2/ period_ms) * 65535)
min_throttle = int ((1/ period_ms) * 65535)
goal_throttle = int ((1.35/ period_ms) * 65535)
# offset for each motor
t1_ofs_throttle = int ((0.035/ period_ms) * 65535)
t2_ofs_throttle = int ((0.0/ period_ms) * 65535)
t3_ofs_throttle = int ((0.01/ period_ms) * 65535)
t4_ofs_throttle = int ((0.01/ period_ms) * 65535)
steps = 100  # Define the number of steps
duty_step = (goal_throttle - min_throttle) // steps

maxThrottleDuration = 4 * 1000 # in ms
hasTilted = False

on = True # from user input. Starts the drone
takeOff = True #phase
landing = False
duty_cycle = min_throttle # starts with min throttle
endHoverTime = 0 # assigns value after takeoff
changeDutyCycle = False
newest_goal = 0
cmd_duty_step = 0
prev_goal = goal_throttle

# PID Controller values
pid_pitch_kp:float = 5#5.5
pid_pitch_ki:float = 0#.4
pid_pitch_kd:float = 0#.5
pid_roll_kp:float = 2#1.0
pid_roll_ki:float = 0#.4
pid_roll_kd:float = 0#.5
pid_yaw_kp:float = 0#1.0
pid_yaw_ki:float = 0#.1
pid_yaw_kd:float = 0#.5

i_limit = 30 # max value i can reach

roll_last_integral:float = 0.0 # default to 0. Accumulates throughout
roll_last_error:float = 0.0 # error value. currently = angle for hovering. Future = goal-current angle
pitch_last_integral:float = 0.0
pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
yaw_last_error:float = 0.0

transition_throttle = int ((1.1/ period_ms) * 65535) # when it switch from motor set duty_cycle to adding pid values

def stopAll() -> None:
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)
    
def constrainThrottle(t1, t2, t3, t4): # constrain throttle to between min to max throttle
    t1 = max(min(t1, max_throttle), min_throttle) # constrain within throttle limits
    t2 = max(min(t2, max_throttle), min_throttle) # constrain within throttle limits
    t3 = max(min(t3, max_throttle), min_throttle) # constrain within throttle limits
    t4 = max(min(t4, max_throttle), min_throttle) # constrain within throttle limits
    return t1,t2,t3,t4

try:

    led.value(1)

    # set all motors to min throttle
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)
    
    sleep (10)
    
    mpu.calibrateGyro()
    
    previous_time = ticks_ms()
    
    count = 0 # to count how many message received. For debugging
    
    while not takeOff: # wait for take off signal
        if rf.existsMessage():
            rf.updateMessage()
            takeOff = rf.getState()
            print("Taking off")
    
    while True:
        
        current_time = ticks_ms()
        
        # Checks for any rf message
        if rf.existsMessage():
            # newest_throttle: similar to 1.1. newest_goal: after calculation like 4320
            rf.updateMessage()
            if rf.getState == 0: # land
                takeOff = False
                landing = True
            else:
                duty_cycle = 1 + rf.getThrottle() / 100 # Calculate new duty_cycle from 1.00 to 2.00
        
        angle = [(rf.getPitch(), rf.getRoll(), rf.getYaw())[i] - mpu.getAngle()[i] for i in range(3)] # get difference between controller's goal and current angle
                
        #loop time
        loop_time = ticks_diff(current_time, previous_time) / 1000.0
        loop_time = max(loop_time, 1e-6)
        
        # pitch PID calc
        pitch_p:float = angle[0] * pid_pitch_kp
        pitch_i:float = pitch_last_integral + (angle[0] * pid_pitch_ki * loop_time)
        pitch_i = max(min(pitch_i, i_limit), -i_limit) # constrain within I-term limits
        pitch_d:float = pid_pitch_kd * (angle[0] - pitch_last_error) / loop_time
        pid_pitch = pitch_p + pitch_i - pitch_d #in original tutorial is p+i+d
        
        # roll PID calc
        roll_p:float = angle[1] * pid_roll_kp
        roll_i:float = roll_last_integral + (angle[1] * pid_roll_ki * loop_time)
        roll_i = max(min(roll_i, i_limit), -i_limit) # constrain within I-term limits
        roll_d:float = pid_roll_kd * (angle[1] - roll_last_error) / loop_time
        pid_roll:float = roll_p + roll_i - roll_d

        # Yaw PID Calculation
        yaw_p = angle[2] * pid_yaw_kp
        yaw_i = yaw_last_integral + angle[2] * pid_yaw_ki * loop_time
        yaw_i = max(min(yaw_i, i_limit), -i_limit)  # Constrain the integral term
        yaw_d = pid_yaw_kd * (angle[2] - yaw_last_error) / loop_time
        pid_yaw = yaw_p + yaw_i + yaw_d
        
        previous_time = current_time
        
        # calculate the throttle for each motor - t1 matches with esc_1
        t1 = t1_ofs_throttle + int(duty_cycle - pid_pitch - pid_roll + pid_yaw)  # Motor 1
        t2 = t2_ofs_throttle + int(duty_cycle + pid_pitch - pid_roll - pid_yaw)  # Motor 2
        t3 = t3_ofs_throttle + int(duty_cycle + pid_pitch + pid_roll + pid_yaw)  # Motor 3
        t4 = t4_ofs_throttle + int(duty_cycle - pid_pitch + pid_roll - pid_yaw)  # Motor 4

        # t1:int = int(duty_cycle - pid_pitch - pid_roll) # - pid_yaw_kp*angle[2]
        # t2:int = int(duty_cycle + pid_pitch - pid_roll) # + pid_yaw_kp*angle[2] 
        # t3:int = int(duty_cycle + pid_pitch + pid_roll) # - pid_yaw_kp*angle[2] 
        # t4:int = int(duty_cycle - pid_pitch + pid_roll) # + pid_yaw_kp*angle[2]
        
        t1,t2,t3,t4 = constrainThrottle(t1,t2,t3,t4)
        
                    
#         Can I delete this?            
#         lines = rcvdFile.readlines()  
#         if lines:
#             last_file_line = float(lines[-1].strip())  # e.g. "1.3"
#             newest_goal = int ((last_file_line/ period_ms) * 65535)
#             print(newest_goal)
            
#                # If there's a difference from the previous value, we set changeDutyCycle = True
#             if prev_goal is None or newest_goal != prev_goal:
#                 changeDutyCycle = True
#                 prev_goal = newest_goal  # store this line as the new "previous"
#             else:
#                 # If it's the same line, do nothing special
#                 pass
#         else:
#             # If the file was empty, we won't change anything
#             newest_goal = goal_throttle
# 
#         # Only do something if changeDutyCycle is True
#         if changeDutyCycle:
#             # Convert newest_goal to an integer throttle
#             cmd_throttle = int(newest_goal)
#             cmd_duty_step = (cmd_throttle - duty_cycle) // steps
#         else:
#             # No change from the previous line
#             cmd_throttle = 0
#             cmd_duty_step = 0
        
        if takeOff: #while taking off

            # set motors to duty cyle - first time at min throttle
            if duty_cycle > transition_throttle: # to prevent having a throttle < min throttle and having a large angle error that might affect take off
                esc_1.duty_u16(t1)
                esc_2.duty_u16(t2)
                esc_3.duty_u16(t3)
                esc_4.duty_u16(t4)
            else: 
                esc_1.duty_u16(duty_cycle + t1_ofs_throttle)
                esc_2.duty_u16(duty_cycle + t2_ofs_throttle)
                esc_3.duty_u16(duty_cycle + t3_ofs_throttle)
                esc_4.duty_u16(duty_cycle + t4_ofs_throttle)
            
            if(duty_cycle >= goal_throttle): # if reached goal throttle
                takeOff = False # Change state 
                # endHoverTime = ticks_add(current_time, maxThrottleDuration) #find endtime
            else:
                duty_cycle += duty_step # if it has not reach goal throttle, keep increasing speed
                        
        elif landing: # Add a condition for the reduction of throttle such that the drone can reducethrottle without immediately going to null and landing
            
            # set motors to duty cyle - first time at goal throttle
            if duty_cycle > transition_throttle:
                esc_1.duty_u16(t1)
                esc_2.duty_u16(t2)
                esc_3.duty_u16(t3)
                esc_4.duty_u16(t4)
            else: 
                esc_1.duty_u16(duty_cycle + t1_ofs_throttle)
                esc_2.duty_u16(duty_cycle + t2_ofs_throttle)
                esc_3.duty_u16(duty_cycle + t3_ofs_throttle)
                esc_4.duty_u16(duty_cycle + t4_ofs_throttle)
                
            if duty_cycle <= min_throttle: ## if passed min throttle
                landing = False
                break # Break out of loop if landed
            else:
                duty_cycle -= duty_step # decrease duty cycle
                        
        # elif changeDutyCycle:
        #     # duty_cycle same during the first run, change afterwards
        #     esc_1.duty_u16(t1)
        #     esc_2.duty_u16(t2)
        #     esc_3.duty_u16(t3)
        #     esc_4.duty_u16(t4)
        #     if (cmd_duty_step>0 and duty_cycle >= newest_goal) or (cmd_duty_step<0 and duty_cycle <= newest_goal): # if passed cmd throttle
        #         changeDutyCycle = False # then it will run else
        #     else:
        #         duty_cycle += cmd_duty_step # change duty cycle
        
        else:
            #stay at same throttle 
            esc_1.duty_u16(t1)
            esc_2.duty_u16(t2)
            esc_3.duty_u16(t3)
            esc_4.duty_u16(t4)
            
            # if(ticks_diff(endHoverTime, current_time) <= 0): # check how long it hovered
            #     landing = True # change to landing if hovered for enough time
        
        # Save state values for next loop
        roll_last_error = angle[1]
        pitch_last_error = angle[0]
        yaw_last_error = angle[2]
        roll_last_integral = roll_i
        pitch_last_integral = pitch_i
        yaw_last_integral = yaw_i
        
        # For debgugging
        #mpu.readData()
        mpu.updateAngle()
        # print(str(duty_cycle))
        #print(str(mpu.getAngle()))
        print(count, [t1,t2,t3,t4], cmd_duty_step, duty_cycle)
#         print(takeOff, changeDutyCycle)
        # write to file - angle then throttle for each motor
        angleFile.write(f"{angle[0]}, {angle[1]}, {angle[2]}, {current_time}\n")
        throttleFile.write(f"{t1}, {t2}, {t3}, {t4}\n")
        
        if(mpu.checkRotationAngle()): 
            stopAll() 
            hasTilted = True
            led.toggle()
            break
        
        sleep(0.005)
        
    stopAll()      
    led.toggle()
    print('Finished')
    angleFile.close()
    throttleFile.close()
    
except KeyboardInterrupt:
    print("Keyboard interrupt")
    esc.duty_u16(0) # didn't fix it yet
    print(esc)
    esc.deinit()


