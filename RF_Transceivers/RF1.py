import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01

#addresses
pipes = (b'\xe1\xf0\xf0\xf0\xf0', b'\xd2\xf0\xf0\xf0\xf0')

#turning on picos built-in LED to indicate that power is on
led = Pin(25, Pin.OUT)
led.value(1)

print('Pico RX Starting')

#setting up nrf24l01 object
spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
csn = Pin(14, mode=Pin.OUT, value=1)
ce = Pin(17, mode=Pin.OUT, value=0)
nrf = NRF24L01(spi, csn, ce, channel=100, payload_size=32)

#opening listening pipe
nrf.open_tx_pipe(pipes[1])
nrf.open_rx_pipe(1, pipes[0])
nrf.start_listening()

print('RX Ready. Waiting for packets...')

while True:
    utime.sleep(1)
    
    #checking for a message on the nrf24l01
    if nrf.any():
        print('Received something!:')
        package = nrf.recv()
        #package_2 = r'package[0:9]'
        msg =package.decode('utf-8')[0:9]
        if msg != "":
            print(msg)
            
        
    