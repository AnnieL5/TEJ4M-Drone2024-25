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

axis = []

vs = 10 #vertical stretch
vt = 4200

vs=1
vt=0

# Open the file in read mode
with open('angleData.txt', 'r') as file:
    # Read each line in the file
    print(file)
    for line in file:
        # Print each line
        values = line.strip().split(',')
        angleX.append(vs*float(values[0])+vt)
        angleY.append(vs*float(values[1])+vt)
        angleZ.append(vs*float(values[2])+vt)
        time.append(vs*float(values[3])+vt)
        axis.append(vt)

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
aZ = np.array(angleZ)


ta = np.array(t1)
tb = np.array(t2)
tc = np.array(t3)
td = np.array(t4)

# figure, axis = plt.subplots(2, 1)


plt.plot(xpoints, aX, color='r', label='pitch')
plt.plot(xpoints, aY, color='b', label='row')
plt.plot(xpoints, aZ, color='k', label='yall')
plt.plot(xpoints, axis, color='g', label='axis')


# plt.plot(xpoints, ta, color='g', label='t1')
# plt.plot(xpoints, tb, color='c', label='t2')
# plt.plot(xpoints, tc, color='m', label='t3')#purple
# plt.plot(xpoints, td, color='y', label='t4')

plt.show()