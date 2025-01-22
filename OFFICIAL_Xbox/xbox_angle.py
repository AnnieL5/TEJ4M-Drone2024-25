import pygame
import serial

# Initialize the variables
pygame.init()
joysticks = []
clock = pygame.time.Clock()
keepPlaying = True

duty_cycle_increment = 0

message = ""
on = 0
pitch = 0
roll = 0
yall = 0

pitchT = 0
rollT = 0
yawT = 0

max_angle = 15

throttle_increment = 0.01

# Initialize the serial connection
arduino = serial.Serial('COM4', 115200)  # Replace 'COM3' with your Arduino's port

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
        # Add the throttle to the current duty cycle

        if event.type == pygame.JOYBUTTONDOWN:
            # on/off
            if joystick.get_button(4):
                on = 1
            if joystick.get_button(5):
                on = 0

            # Change duty cycle
            if joystick.get_button(3):
                duty_cycle_increment += throttle_increment
            elif joystick.get_button(1):
                duty_cycle_increment -= throttle_increment
            # print(str(on)+str(current_duty_cycle))
        
        elif event.type == pygame.JOYAXISMOTION:
            pitch = joystick.get_axis(1) # Forward positive
            roll = joystick.get_axis(0) # Left positive
            yaw = - joystick.get_axis(2)# Left positive

            pitchT = mapFromTo(pitch, -1, 1, -max_angle, max_angle)
            rollT = mapFromTo(roll, -1, 1, -max_angle, max_angle)
            yawT = mapFromTo(yaw, -1, 1, -max_angle, max_angle)

    message = f'{on}, {pitchT:.0f}, {rollT:.0f}, {yawT:.0f}, {duty_cycle_increment*100:.0f}' 
    print(message)
    
    # send to the Arduino
    arduino.write(message.encode())
