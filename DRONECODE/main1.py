from machine import Pin, PWM
from time import sleep
from MPU_data import MPU6050DATA

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

maxThrottleTime = 2
hasTilted = False

def stopAll():
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)

try:

    led.value(1)
    
    esc_1.duty_u16(min_throttle)
    esc_2.duty_u16(min_throttle)
    esc_3.duty_u16(min_throttle)
    esc_4.duty_u16(min_throttle)
    
    sleep (10)
    
    for i in range(1):
        for duty_cycle in range(min_throttle, goal_throttle + duty_step, duty_step): #check notes document for why max throttle + duty_step
            esc_1.duty_u16(duty_cycle)
            esc_2.duty_u16(duty_cycle)
            esc_3.duty_u16(duty_cycle)
            esc_4.duty_u16(duty_cycle + int(add_throttle))
            if(mpu.checkRotation()): 
                stopAll() #Stop all motors if it tilts
                hasTilted = True
                break
                
            sleep(0.005)
        
        if(hasTilted == False):
            esc_1.duty_u16(goal_throttle)
            esc_2.duty_u16(goal_throttle)
            esc_3.duty_u16(goal_throttle)
            esc_4.duty_u16(goal_throttle + add_throttle)
        
            for i in range(maxThrottleTime*4):
                mpu.readData()
                if(mpu.checkRotation()): 
                    stopAll() #Stop all motors if it tilts
                    hasTilted = True
                    break
                sleep(0.25)
        
        if(hasTilted == False):
            for duty_cycle in range(goal_throttle + duty_step, min_throttle, -duty_step):
                esc_1.duty_u16(duty_cycle)
                esc_2.duty_u16(duty_cycle)
                esc_3.duty_u16(duty_cycle)
                esc_4.duty_u16(duty_cycle + add_throttle)
                
                if(mpu.checkRotation()): 
                    stopAll() #Stop all motors if it tilts
                    hasTilted = True
                    break
            
                sleep(0.005)
            
        led.toggle()
    
except KeyboardInterrupt:
    print("Keyboard interrupt")
    esc.duty_u16(0)
    print(esc)
    esc.deinit()
