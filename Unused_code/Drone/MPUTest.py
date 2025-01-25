import MPU_data

mpu = MPU6050DATA(0, 12, 13)
mpu.calibrateGyro()
# # print(str(mpu.getAngle()))
# 
while True:
#     #mpu.readData()
    mpu.updateAngle()
    angle = mpu.getAngle()
    print(str(angle))
    sleep(0.1)
    print(str(mpu.checkRotationAngle()))
# if len(devices) == 0:
#     print("No I2C devices found.")
# else:
#     print("I2C devices found at addresses:")
#     for d in devices:
#         print(" - Address:", hex(d))