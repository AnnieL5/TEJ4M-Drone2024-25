import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01
import os

class RFClass: 
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
#     past_msg = "0000000000000" # Default
    
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

        # os.remove('rcvd.txt')
        # f = open('rcvd.txt', 'a')

        print('RX Ready. Waiting for packets...')

    def existsMessage(self) -> bool:
        #checking for a message on the nrf24l01
        if self.nrf.any():
            return True
        else:
            return False
        
    def updateMessage(self) -> str:
        print('Received something:')
        package = self.nrf.recv()
        #package_2 = r'package[0:9]'
        print(package)
        try: 
            if package.strip(b'\x00'):  # Remove padding bytes and check if anything is left
                msg = package.decode('utf-8')[0:self.msgLength]
                try:
                    self.assignValues(msg)
                    self.msg = msg
                    # self.past_msg = msg
                    print(f"Decoded message: {self.msg}")
                    return msg
                except (UnicodeError, ValueError, TypeError):
                    print("Decoded. Assigning Filed")
                    self.assignValues(self.msg)
                    # self.assignValues(self.past_msg)
            else:
                print("Received empty or padding data.")
        except (UnicodeError, ValueError, TypeError):
            print("Decoding failed")
        #Python doesn't neqed the null terminator but to 32 ensures we don't accidentally truncate any data that was meant to be sent. 
    def assignValues(self, msg: str):
        var = msg.split(', ')
        # state, pritch, roll, yaw, throttle, * = msg.split(', ')
        
        self.state = bool(var[0])
        self.pitch = int(var[1])
        self.roll = int(var[2])
        self.yaw = int(var[3])
        self.throttle = int(var[4])

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
#         
# 
rf = RFClass()

while True:
    utime.sleep(0.1)

    if rf.existsMessage():
        msg = rf.updateMessage()
        print(rf.getMessage())
        print(f'on: {rf.getState()}, pitch: {rf.getPitch()}, roll: {rf.getRoll()}, yaw: {rf.getYaw()}, throttle: {rf.getThrottle()}')
#         for char in msg:
#             print(f"digit: {char}")
#         print(int(msg[3]))
#     # open file in append mode and write the received message
#         if(msg[0]== "c"):
#             print('here')
#             break
#         else:
#             with open('rcvd.txt', 'a') as f: #automatically closes file after writing 
#                 f.write(msg[0:4] + '\n')
#             print('Here2')

print('finish')  