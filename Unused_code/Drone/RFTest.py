import RFClass
import utime

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

# print('finish')  