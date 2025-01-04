import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01
import os

class RFClass: 
    #addresses
    pipes = (b'\xe1\xf0\xf0\xf0\xf0', b'\xd2\xf0\xf0\xf0\xf0')
    
    msgLength = 31

    #setting up nrf24l01 object
    spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
    #Keep csn & ce pins the same - do not change
    csn = Pin(14, mode=Pin.OUT, value=1)
    ce = Pin(17, mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, channel=100, payload_size=(msgLength+1))

    
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
        
    def getMessage(self) -> str:
        print('Received something:')
        package = self.nrf.recv()
        #package_2 = r'package[0:9]'
        print(package)
        msg=package.decode('utf-8')[0:self.msgLength] #type string
        #Python doesn't neqed the null terminator but to 32 ensures we don't accidentally truncate any data that was meant to be sent. 
        
        return msg        
        
        #print(f.read())
#         if msg.strip() == "":
#             print("Empty Message")
#         else:
#             print(msg)

rf = RFClass()

while True:
    utime.sleep(1)
    
    if rf.existsMessage():
        msg = rf.getMessage()
    # open file in append mode and write the received message
        if(msg[0]== "c"):
            print('here')
            break
        else:
            with open('rcvd.txt', 'a') as f: #automatically closes file after writing 
                f.write(msg[0:4] + '\n')
            print('Here2')
    
print('finish')  
