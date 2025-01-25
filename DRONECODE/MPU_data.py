import machine
from machine import Pin, I2C
from time import ticks_ms, ticks_add, ticks_diff, sleep
import MPU6050

class MPU6050DATA():
    
    gyroOFS = [0,0,0]
    acceOFS = [0,0,0]
     
    gyroAngle = [0,0,0]
    pastTime = 0
    
    maxSpeed = 20
    maxAngle = 15#previous 25 
    
    def __init__(self, id, sda, scl):
        # Set up the I2C interfac
        self.i2c = machine.I2C(id, sda=machine.Pin(sda), scl=machine.Pin(scl)) #0, sda=machine.Pin(12), scl=machine.Pin(13)

        # Set up the MPU6050 class 
        self.mpu = MPU6050.MPU6050(self.i2c)

        # wake up the MPU6050 from sleep
        self.mpu.wake()
        
        # Turning the low pass filter setting to level 5 (out of 6)
        self.mpu.write_lpf_range(5) 
        
    # Read and Print the data from the MPU6050
    def readData(self) -> None:
        print("Gyro: " + str(self.getGyro()) + ", Accel: " + str(self.getAcce()))
        sleep(0.1)
    
    # Get the gyro data
    def getGyro(self) -> list: # with offset
        gyro = list(self.mpu.read_gyro_data())
        return [gyro[i] - self.gyroOFS[i] for i in range(3)]
    
    # Get the accelerometer data
    def getAcce(self) -> list: # with offset
        accel = list(self.mpu.read_accel_data())# - self.acceOFS
        return [accel[i] - self.acceOFS[i] for i in range(3)]
    
    # Get the current angle
    def getAngle(self) -> list:
        return self.gyroAngle
    
    # Calibrate the gyro by averaging 1000 readings
    def calibrateGyro(self):
        for i in range(1000):
            data = list(self.mpu.read_gyro_data())
            self.gyroOFS = [self.gyroOFS[j] + data[j] for j in range(3)]
            
        self.gyroOFS = [self.gyroOFS[i] / 1000.0 for i in range(3)]
        
        self.gyroAngle = [0,0,0]
        self.startTime = ticks_ms()
        self.aggregateTime = 0 # in ms
        
    # Calibrate the accelerometer
    def calibrateAcce(self):
        self.acceOFS = self.mpu.read_gyro_data()
        
    # Check if the rotation speed is higher than the limit
    def checkRotationSpeed(self):
        for i in self.getGyro():
            if(abs(i)> self.maxSpeed):
                return True
        return False
    
    # Check if the rotation angle is higher than the limit
    def checkRotationAngle(self):
        for i in self.getAngle()[0:2]:
            if(abs(i)> self.maxAngle):
                return True
        return False
    
    # Update the angle
    def updateAngle(self):
        t = ticks_ms()
        time = ticks_diff(t, self.startTime) - self.aggregateTime
        self.gyroAngle = [self.gyroAngle[i] + time /1000 * self.getGyro()[i] for i in range(3)] # gives value in deg/s
        self.aggregateTime += time
        
        sleep(0.005)
    