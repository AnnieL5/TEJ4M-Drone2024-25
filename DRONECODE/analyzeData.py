import matplotlib.pyplot as plt
import numpy as np

angleX = []
angleY = []
angleZ = []
time = []

t1 = []
t2 = []
t3 = []
t4 = []

vs = 100 #vertical stretch

# Open the file in read mode
with open('angleData.txt', 'r') as file:
    # Read each line in the file
    for line in file:
        # Print each line
        values = line.strip().split(',')
        angleX.append(vs*float(values[0])+3000)
        angleY.append(vs*float(values[1])+3000)
        angleZ.append(vs*float(values[2])+3000)
        time.append(vs*float(values[3])+3000)

# Open the file in read mode
with open('throttleData.txt', 'r') as file:
    # Read each line in the file
    for line in file:
        # Print each line
        values = line.strip().split(',')
        t1.append(int(values[0]))
        t2.append(int(values[1]))
        t3.append(int(values[2]))
        t4.append(int(values[3]))

xpoints = np.array(time)
aX = np.array(angleX)
aY = np.array(angleY)

ta = np.array(t1)
tb = np.array(t2)
tc = np.array(t3)
td = np.array(t4)

plt.plot(xpoints, aX, color='r', label='pitch')
plt.plot(xpoints, aY, color='b', label='row')

plt.plot(xpoints, ta, color='g', label='t1')
plt.plot(xpoints, tb, color='c', label='t2')
plt.plot(xpoints, tc, color='m', label='t3')
plt.plot(xpoints, td, color='y', label='t4')

plt.show()