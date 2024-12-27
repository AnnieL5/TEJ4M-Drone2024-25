from machine import Pin, PWM
from time import sleep, ticks_ms, ticks_add, ticks_diff
from MPU_data import MPU6050DATA

# From 12.21
# - with a while loop
# - detects angle instead of angular speed

led = Pin(25, Pin.OUT)

mpu = MPU6050DATA(id=0, sda=12, scl=13)

esc_1 = PWM(Pin(14))
esc_2 = PWM(Pin(2))
esc_3 = PWM(Pin(16))
esc_4 = PWM(Pin(28))

esc_1.freq(50)
esc_2.freq(50)
esc_3.freq(50)
esc_4.freq(50)

period_ms = 20
min_throttle = int ((1/ period_ms) * 65535)
goal_throttle = int ((1.35/ period_ms) * 65535)
add_throttle = int ((0.01/ period_ms) * 65535)
#improvements
steps = 100  # Define the number of steps
duty_step = (goal_throttle - min_throttle) // steps

maxThrottleDuration = 2 * 1000 # in ms
hasTilted = False

on = True
takeOff = True
landing = False
duty_cycle = min_throttle
endHoverTime = 0

def stopAll():
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)

try:

    led.value(1)
    
    # set all motors to min throttle
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)
    
    mpu.calibrateGyro()
    
    sleep (10)
    
    while(on):
        if(takeOff): #while taking off
        #for duty_cycle in range(min_throttle, goal_throttle + duty_step, duty_step): #check notes document for why max throttle + duty_step
            # set motors to duty cyle - first time at min throttle
            esc_1.duty_u16(duty_cycle)
            esc_2.duty_u16(duty_cycle)
            esc_3.duty_u16(duty_cycle)
            esc_4.duty_u16(duty_cycle + int(add_throttle))
            
            if(duty_cycle >= goal_throttle): # if reached goal throttle
                takeOff = False # Change state 
                endHoverTime = ticks_add(ticks_ms(), maxThrottleDuration)
            else:
                duty_cycle += duty_step # if it has not reach goal throttle, keep increasing speed
                        
        elif(landing):
            # for duty_cycle in range(goal_throttle + duty_step, min_throttle, -duty_step):
            # set motors to duty cyle - first time at goal throttle
            esc_1.duty_u16(duty_cycle)
            esc_2.duty_u16(duty_cycle)
            esc_3.duty_u16(duty_cycle)
            esc_4.duty_u16(duty_cycle + add_throttle)
                
            if(duty_cycle <= min_throttle): ## if passed min throttle
                landing = False
                break # Break out of loop if landed
            else:
                duty_cycle -= duty_step # decrease duty cycle
                        
        else:
            # esc_1.duty_u16(goal_throttle)
            # esc_2.duty_u16(goal_throttle)
            # esc_3.duty_u16(goal_throttle)
            # esc_4.duty_u16(goal_throttle + add_throttle)
        
            # for i in range(maxThrottleDuration*4):
                        
            if(ticks_diff(endHoverTime, ticks_ms()) <= 0): # check how long it hovered
                landing = True # change to landing if hovered for enough time
        
        #Stop all motors if it tilts
        #mpu.readData()
        mpu.updateAngle()
        print(str(duty_cycle))
        print(str(mpu.getAngle()))
        if(mpu.checkRotationAngle()): 
                stopAll() 
                hasTilted = True
                break
        
        sleep(0.05)
            
    led.toggle()
    print('Finished')
except KeyboardInterrupt:
    print("Keyboard interrupt")
    esc.duty_u16(0)
    print(esc)
    esc.deinit()

