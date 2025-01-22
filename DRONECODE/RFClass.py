import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01
import os

class RFClass: 
    #addresses
    pipes = (b'\xe1\xf0\xf0\xf0\xf0', b'\xd2\xf0\xf0\xf0\xf0')
    
    msgLength = 4 #one less

    #setting up nrf24l01 object
    spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))

    #Keep csn & ce pins the same - do not change
    csn = Pin(14, mode=Pin.OUT, value=1)
    ce = Pin(17, mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, channel=100, payload_size=(32))
    
    message = "1.00" # Default
    
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

    def existsMessage(self) -> bool:
        #checking for a message on the nrf24l01
        if self.nrf.any():
            return True
        else:
            return False
        
    def getMessage(self):
        print('Received something:')
        package = self.nrf.recv()
        print(package)
        try: 
            if package.strip(b'\x00'):  # Remove padding bytes and check if anything is left
                msg = package.decode('utf-8')
                print(f"Decoded message: {msg}")
                return msg
            else:
                print("Received empty or padding data.")
        except (UnicodeError, ValueError, TypeError):
            print("Decoding failed")