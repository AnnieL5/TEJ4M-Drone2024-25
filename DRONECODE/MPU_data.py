import machine
from machine import Pin, I2C
from time import ticks_ms, ticks_add, ticks_diff, sleep
import MPU6050

class MPU6050DATA():
    
    #pOFS, yOFS, rOFS = 0
    #gyroOFS = [pOFS, yOFS, rOFS]
    
    gyroOFS = [0,0,0]
    acceOFS = [0,0,0]
     
    gyroAngle = [0,0,0]
    pastTime = 0
    
    maxSpeed = 20
    maxAngle = 20
    
    def __init__(self, id, sda, scl):
        # Set up the I2C interfac
        self.i2c = machine.I2C(id, sda=machine.Pin(sda), scl=machine.Pin(scl)) #0, sda=machine.Pin(12), scl=machine.Pin(13)

        # Set up the MPU6050 class 
        self.mpu = MPU6050.MPU6050(self.i2c)

        # devices = i2c.scan()

        # wake up the MPU6050 from sleep
        self.mpu.wake()
        
        self.mpu.write_lpf_range(5) # Turning the low pass filter setting to level 5 (out of 6)
        
#         self.gyroOFS = [0,0,0]
#         self.acceOFS = [0,0,0]
#          
#         self.gyroAngle = [0,0,0]
#         self.pastTime = 0
        # print(str(self.mpu.read_gyro_range()))
    #co/ntinuously print the data
    # while True:
    def readData(self) -> None:
#         gyro = list(self.mpu.read_gyro_data())
#         difGyro = [gyro[i] - self.gyroOFS[i] for i in range(3)]
#         accel = list(self.mpu.read_accel_data())# - self.acceOFS
#         difAccel = [accel[i] - self.acceOFS[i] for i in range(3)]
        
        print("Gyro: " + str(self.getGyro()) + ", Accel: " + str(self.getAcce()))
        sleep(0.1)
    
    def getGyro(self) -> list: # with offset
        gyro = list(self.mpu.read_gyro_data())
        return [gyro[i] - self.gyroOFS[i] for i in range(3)]
    
    def getAcce(self) -> list: # with offset
        accel = list(self.mpu.read_accel_data())# - self.acceOFS
        return [accel[i] - self.acceOFS[i] for i in range(3)]
    
    def getAngle(self) -> list:
        return self.gyroAngle
    
    def calibrateGyro(self):
        for i in range(100):
            data = list(self.mpu.read_gyro_data())
            self.gyroOFS = [self.gyroOFS[j] + data[j] for j in range(3)]
            
        self.gyroOFS = [self.gyroOFS[i] / 100.0 for i in range(3)]
        # print(str(self.gyroOFS))
        
        self.gyroAngle = [0,0,0]
        #self.pastTime = ticks_ms()
        self.startTime = ticks_ms()
        self.aggregateTime = 0 # in ms
        
    def calibrateAcce(self):
        self.acceOFS = self.mpu.read_gyro_data()
        
    def checkRotationSpeed(self):
        for i in self.getGyro():
            if(abs(i)> self.maxSpeed):
                return True
        return False
    
    def checkRotationAngle(self):
        for i in self.getAngle()[0:2]:
            if(abs(i)> self.maxAngle):
                return True
        return False
    
    def updateAngle(self):
        t = ticks_ms()
        time = ticks_diff(t, self.startTime) - self.aggregateTime
        self.gyroAngle = [self.gyroAngle[i] + time /1000 * self.getGyro()[i] for i in range(3)] # gives value in deg/s
        self.aggregateTime += time
        
        sleep(0.005)
    
# mpu = MPU6050DATA(0, 12, 13)
# mpu.calibrateGyro()
# # # # print(str(mpu.getAngle()))
# # # 
# while True:
# #     #mpu.readData()
#     mpu.updateAngle()
#     angle = mpu.getAngle()
#     print(str(angle))
#     sleep(0.1)
#     print(str(mpu.checkRotationAngle()))
# if len(devices) == 0:
#     print("No I2C devices found.")
# else:
#     print("I2C devices found at addresses:")
#     for d in devices:
#         print(" - Address:", hex(d))