import pygame
import serial

pygame.init()
joysticks = []
clock = pygame.time.Clock()
keepPlaying = True

current_duty_cycle = 1.35

message = ""
on = 0
t1 = current_duty_cycle
t2 = current_duty_cycle
t3 = current_duty_cycle
t4 = current_duty_cycle

max_throttle = 1.8
min_throttle = 1.3
mid_throttle = 1.55

throttle_increment = 0.01
count = 0

arduino = serial.Serial('COM3', 115200)  # Replace 'COM3' with your Arduino's port

def mapFromTo(x,a,b,c,d):
   y=(x-a)/(b-a)*(d-c)+c
   return y

# for al the connected joysticks
for i in range(0, pygame.joystick.get_count()):
    # create an Joystick object in our list
    joysticks.append(pygame.joystick.Joystick(i))
    # initialize the appended joystick (-1 means last array item)
    joysticks[-1].init()
    # print a statement telling what the name of the controller is
    print ("Detected joystick "),joysticks[-1].get_name(),"'"
joystick = joysticks[0]

while keepPlaying:
    clock.tick(10)
    
    message = ""

    for event in pygame.event.get():
        # The 0 button is the 'a' button, 1 is the 'b' button, 2 is the 'x' button, 3 is the 'y' button
        # print(str(joystick.get_button(3))+str(joystick.get_button(1))+str(joystick.get_button(4))+str(joystick.get_button(5)))
        # on/off
                # Add the throttle to the current duty cycle
        t1 = current_duty_cycle - 1
        t2 = current_duty_cycle - 1
        t3 = current_duty_cycle - 1
        t4 = current_duty_cycle - 1

        if event.type == pygame.JOYBUTTONDOWN:
            if joystick.get_button(4):
                on = 1
            if joystick.get_button(5):
                on = 0

            # Change duty cycle
            if joystick.get_button(3):
                current_duty_cycle += throttle_increment
            elif joystick.get_button(1):
                current_duty_cycle -= throttle_increment
            # print(str(on)+str(current_duty_cycle))
        
        elif event.type == pygame.JOYAXISMOTION:
            pitch = - joystick.get_axis(1) # Forward positive
            roll = - joystick.get_axis(0) # Left positive
            yaw = - joystick.get_axis(2)# Left positive


            max_diff = current_duty_cycle - 1 if current_duty_cycle < mid_throttle else 2 - current_duty_cycle
            pitchT = mapFromTo(pitch, -1, 1, -max_diff, max_diff)
            rollT = mapFromTo(roll, -1, 1, -max_diff, max_diff)
            yawT = mapFromTo(yaw, -1, 1, -max_diff, max_diff)

            # Calculate the addition throttle for each motor
            t1 += - pitchT - rollT + yawT
            t2 += + pitchT - rollT - yawT
            t3 += + pitchT + rollT + yawT
            t4 += - pitchT + rollT - yawT

        # constrain within throttle limits
        t1 = max(min(t1, max_throttle -1), min_throttle-1) 
        t2 = max(min(t2, max_throttle-1), min_throttle-1) # constrain within throttle limits
        t3 = max(min(t3, max_throttle-1), min_throttle-1) # constrain within throttle limits
        t4 = max(min(t4, max_throttle-1), min_throttle-1)

            # print(f'{pitch}, {roll}, {yaw}')
            # print(f'{t1}, {t2}, {t3}, {t4}')
    message = f'{on}, {t1*100:.0f}, {t2*100:.0f}, {t3*100:.0f}, {t4*100:.0f}' 
    print(message)
    
    arduino.write(message.encode())


    # for event in pygame.event.get():
    #     # The 0 button is the 'a' button, 1 is the 'b' button, 2 is the 'x' button, 3 is the 'y' button
    #     if(event.type == 1536):
    #         for i in range(4):
    #             value = joystick.get_axis(i)
    #             if abs(value)>0.05:
    #                 print(f"Axis {i}: {joystick.get_axis(i)}")
    #     else:
    #         for i in range(8):
    #             value = joystick.get_button(i)
    #             if value != 0:
    #                 print(f"Button {i}: {joystick.get_button(i)}")
