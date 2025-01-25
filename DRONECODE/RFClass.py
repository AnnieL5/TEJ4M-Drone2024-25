import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01
import os

class RFClass: 
    
    # Initial values
    #addresses
    pipes = (b'\xe1\xf0\xf0\xf0\xf0', b'\xd2\xf0\xf0\xf0\xf0')
    
    msgLength = 23 #one less

    #setting up nrf24l01 object
    spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
    #Keep csn & ce pins the same - do not change
    csn = Pin(14, mode=Pin.OUT, value=1)
    ce = Pin(17, mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, channel=100, payload_size=(32))
    
    msg = "0000000000000" # Default
    past_msg = "0000000000000" # Default
    
    state = False
    pitch = 0
    roll = 0
    yaw = 0
    throttle = 0
    
    def __init__(self):

        #turning on picos built-in LED to indicate that power is on
        led = Pin(25, Pin.OUT)
        led.value(1)

        print('Pico RX Starting')

        #opening listening pipe
        self.nrf.open_tx_pipe(self.pipes[1])
        self.nrf.open_rx_pipe(1, self.pipes[0])
        self.nrf.start_listening()

        print('RX Ready. Waiting for packets...')

    # Check if a message exists
    def existsMessage(self) -> bool:
        #checking for a message on the nrf24l01
        if self.nrf.any():
            return True
        else:
            return False
        
    # Update the message. Only returns the message if it is not empty, and in the proper format. Else None
    def updateMessage(self) -> str:
        print('Received something:')
        package = self.nrf.recv()
        print(package) # For debugging
        try: 
            if package.strip(b'\x00'):  # Remove padding bytes and check if anything is left
                msg = package.decode('utf-8')[0:self.msgLength]
                try: # Try to understand the message. Go to except if it is not in the correct format
                    self.assignValues(msg)
                    self.msg = msg
                    self.past_msg = msg
                    print(f"Decoded message: {self.msg}")
                    return msg
                except (UnicodeError, ValueError, TypeError, IndexError):
                    print("Decoded. Assigning Filed")
                    self.assignValues(self.past_msg)
                    # self.assignValues(self.past_msg)
            else:
                print("Received empty or padding data.")
        except (UnicodeError, ValueError, TypeError, IndexError):
            print("Decoding failed")
    
    # Assign values to the class variables from the message
    def assignValues(self, msg: str):
        var = msg.split(', ') # Split the message into its components
        
        self.state = bool(int(var[0]))
        self.pitch = int(var[1])
        self.roll = int(var[2])
        self.yaw = int(var[3])
        self.throttle = int(var[4])

    # Return Class variables
    
    def getMessage(self) -> str:
        return self.msg

    def getState(self) -> bool:
        return self.state

    def getPitch(self) -> int:
        return self.pitch

    def getRoll(self) -> int:
        return self.roll

    def getYaw(self) -> int:
        return self.yaw

    def getThrottle(self) -> int:
        return self.throttle 