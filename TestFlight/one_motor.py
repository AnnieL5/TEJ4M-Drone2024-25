from machine import Pin, PWM
from time import sleep

esc = PWM(Pin(28))
led = Pin(25, Pin.OUT)

esc.freq(50)

period_ms = 20
min_throttle = int ((1/ period_ms) * 65535)
max_throttle = int ((1.3/ period_ms) * 65535)
#improvements
steps = 100  # Define the number of steps
duty_step = (max_throttle - min_throttle) // steps

try:
    
    led.value(1)
    
    esc.duty_u16(min_throttle)
    sleep (6)
    
      # Increase the duty cycle gradually
    for duty_cycle in range(min_throttle, max_throttle + duty_step, duty_step): #check notes document for why max throttle + duty_step
        esc.duty_u16(duty_cycle)
        sleep(0.005) #sleep time is really small so it seems like the motors aren't stopping 
        
      # Decrease the duty cycle gradually
    for duty_cycle in range(max_throttle + duty_step, min_throttle, -duty_step):
        esc.duty_u16(duty_cycle)
        sleep(0.005)
        #continous beeping after stopping program 
        
except KeyboardInterrupt:
    print("Keyboard interrupt")
    esc.duty_u16(0)
    print(esc)
    esc.deinit()