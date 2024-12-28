from machine import Pin, PWM
from time import sleep, ticks_ms, ticks_add, ticks_diff
from MPU_data import MPU6050DATA

# From 12.21
# - with a while loop
# - detects angle instead of angular speed
# Future: add error value. To be controller - gyro data
# 		  add pid for yaw & kd

led = Pin(25, Pin.OUT)

mpu = MPU6050DATA(id=0, sda=12, scl=13)

esc_1 = PWM(Pin(14)) # Top Left - weaker one
esc_2 = PWM(Pin(2)) # bottom left - ccw
esc_3 = PWM(Pin(28)) # Bottom right
esc_4 = PWM(Pin(16)) # Top right - ccw

esc_1.freq(50)
esc_2.freq(50)
esc_3.freq(50)
esc_4.freq(50)

period_ms = 20
max_throttle = int ((2/ period_ms) * 65535)
min_throttle = int ((1/ period_ms) * 65535)
goal_throttle = int ((1.34/ period_ms) * 65535)
add_throttle = int ((0.03/ period_ms) * 65535)
#improvements
steps = 100  # Define the number of steps
duty_step = (goal_throttle - min_throttle) // steps

maxThrottleDuration = 5 * 1000 # in ms
hasTilted = False

on = True
takeOff = True
landing = False
duty_cycle = min_throttle
endHoverTime = 0

# PID Controller values
pid_roll_kp:float = 4.5
pid_roll_ki:float = 0.002
# pid_roll_kd:float = 0.00002571429
pid_pitch_kp:float = pid_roll_kp
pid_pitch_ki:float = pid_roll_ki
# pid_pitch_kd:float = pid_roll_kd
pid_yaw_kp:float = pid_roll_kp
pid_yaw_ki:float = 0.003428571
# pid_yaw_kd:float = 0.0

i_limit = 30

roll_last_integral:float = 0.0
#roll_last_error:float = 0.0
pitch_last_integral:float = 0.0
#pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
# yaw_last_error:float = 0.0

transition_throttle = int ((1.1/ period_ms) * 65535) # when it switch from motor set duty_cycle to adding pid values

def stopAll() -> None:
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)

def setThrottle(throttle) -> None:
    esc_1.duty_u16(duty_cycle + add_throttle)
    esc_2.duty_u16(duty_cycle)
    esc_3.duty_u16(duty_cycle)
    esc_4.duty_u16(duty_cycle)
    
def constrainThrottle(t1, t2, t3, t4):
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

    while on:
        # led.value(0)
        
        angle = mpu.getAngle()
        
        # pitch PID calc
        pitch_p:float = angle[0] * pid_pitch_kp
        pitch_i:float = pitch_last_integral + (angle[0] * pid_pitch_ki)
        pitch_i = max(min(pitch_i, i_limit), -i_limit) # constrain within I-term limits
        # pitch_d:float = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
        pid_pitch = pitch_p + pitch_i # + pitch_d
        
         # roll PID calc
        roll_p:float = angle[1] * pid_roll_kp
        roll_i:float = roll_last_integral + (angle[1] * pid_roll_ki)
        roll_i = max(min(roll_i, i_limit), -i_limit) # constrain within I-term limits
        # roll_d:float = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
        pid_roll:float = roll_p + roll_i # + roll_d
        
        t1:int = int(duty_cycle - pid_pitch - pid_roll) # - pid_yaw_kp*angle[2]
        t2:int = int(duty_cycle + pid_pitch - pid_roll) # + pid_yaw_kp*angle[2] 
        t3:int = int(duty_cycle + pid_pitch + pid_roll) # - pid_yaw_kp*angle[2] 
        t4:int = int(duty_cycle - pid_pitch + pid_roll) # + pid_yaw_kp*angle[2]
        
        t1,t2,t3,t4 = constrainThrottle(t1,t2,t3,t4)

        if takeOff: #while taking off

            # set motors to duty cyle - first time at min throttle
            if duty_cycle > transition_throttle: # to prevent having a throttle < min throttle
                esc_1.duty_u16(t1 + add_throttle)
                esc_2.duty_u16(t2)
                esc_3.duty_u16(t3)
                esc_4.duty_u16(t4)
            else: 
                esc_1.duty_u16(duty_cycle + add_throttle)
                esc_2.duty_u16(duty_cycle)
                esc_3.duty_u16(duty_cycle)
                esc_4.duty_u16(duty_cycle)
            
            if(duty_cycle >= goal_throttle): # if reached goal throttle
                takeOff = False # Change state 
                endHoverTime = ticks_add(ticks_ms(), maxThrottleDuration)
            else:
                duty_cycle += duty_step # if it has not reach goal throttle, keep increasing speed
                        
        elif landing:
            # set motors to duty cyle - first time at goal throttle
            if duty_cycle > transition_throttle:
                esc_1.duty_u16(t1 + add_throttle)
                esc_2.duty_u16(t2)
                esc_3.duty_u16(t3)
                esc_4.duty_u16(t4)
            else: 
                esc_1.duty_u16(duty_cycle + add_throttle)
                esc_2.duty_u16(duty_cycle)
                esc_3.duty_u16(duty_cycle)
                esc_4.duty_u16(duty_cycle)
                
            if duty_cycle <= min_throttle: ## if passed min throttle
                landing = False
                break # Break out of loop if landed
            else:
                duty_cycle -= duty_step # decrease duty cycle
                        
        else:
            esc_1.duty_u16(t1 + add_throttle)
            esc_2.duty_u16(t2)
            esc_3.duty_u16(t3)
            esc_4.duty_u16(t4)
        
            if(ticks_diff(endHoverTime, ticks_ms()) <= 0): # check how long it hovered
                landing = True # change to landing if hovered for enough time
        
        # Save state values for next loop
#         roll_last_error = error_rate_roll
#         pitch_last_error = error_rate_pitch
#         yaw_last_error = error_rate_yaw
        roll_last_integral = roll_i
        pitch_last_integral = pitch_i
        #yaw_last_integral = yaw_i
        
        #Stop all motors if it tilts
        #mpu.readData()
        mpu.updateAngle()
        # print(str(duty_cycle))
        # print(str(mpu.getAngle()))
        print([t1,t2,t3,t4])
        if(mpu.checkRotationAngle()): 
                stopAll() 
                hasTilted = True
                led.toggle()
                break
        
        sleep(0.01)
        
    stopAll()        
    led.toggle()
    print('Finished')
    
except KeyboardInterrupt:
    print("Keyboard interrupt")
    esc.duty_u16(0)
    print(esc)
    esc.deinit()
