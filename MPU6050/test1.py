import machine
from machine import Pin
import MPU6050
import time

vcc = Pin(14, Pin.OUT, Pin.PULL_UP)
i2c = machine.I2C(0, sda=machine.Pin(12), scl=machine.Pin(13)) # creating the object that allows for I2C communication in MicroPython
imu = MPU6050.MPU6050(i2c) # passing the i2c object to the MPU6050 class above. This class will handle all communications
imu.wake() # wakes up the MPU-6050 (it may have been in sleep mode)
gyro_data = imu.read_gyro_data()
print(gyro_data) # (0.346823, -0.198345, 0.023958)

time.sleep(0.1)