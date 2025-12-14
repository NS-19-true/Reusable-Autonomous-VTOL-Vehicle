"""
Python visualization of 3D trajectory using Matplotlib (animated).
Expects serial lines with 8 comma-separated values:
  columns[5..7] = N, E, D position in meters (NED frame)
Set the COM port and baud as needed; defaults to `COM10` at 2000000.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

import serial


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
line, = ax.plot([], [], [], 'b-')  # Modify the marker and line style as desired
# define numpy arrays x, y, z which will append the values of N, E, D respectively

x = np.array([])
y = np.array([])
z = np.array([])

N = 0
E = 0
D = 0


ser = serial.Serial('COM10', 2000000, timeout=1)
ser.flushInput()
line_data = ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
columns = line_data.split(",")  # split line into columns using comma as delimiter

# wait until we get a valid line
while len(columns) != 8:
    ser.flushInput()
    line_data = ser.readline().decode().rstrip()
    columns = line_data.split(",")

if len(columns) == 8:  # make sure there are three columns
    try:
        N = float(columns[5])
        E = float(columns[6])
        D = float(columns[7])
        
        x = np.append(x, N)
        y = np.append(y, E)
        z = np.append(z, D)
   
    except:
        print("Invalid line: " + line)


def update(frame):
    # Generate new coordinates for each frame
    global x, y, z
    global N, E, D

    ser.flushInput()
    line_data = ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
    columns = line_data.split(",")  # split line into columns using comma as delimiter

    while len(columns) != 8:
        ser.flushInput()
        line_data = ser.readline().decode().rstrip()
        columns = line_data.split(",")
    
    if len(columns) == 8:  # make sure there are three columns
        try:
            N = float(columns[5])
            E = float(columns[6])
            D = float(columns[7])
            
            x = np.append(x, N)
            y = np.append(y, E)
            z = np.append(z, D)
    
        except:
            print("Invalid line: " + line)

    line.set_data(x, y)
    line.set_3d_properties(z)
    return line,
ani = FuncAnimation(fig, update, frames=range(10000), interval=10, blit=True)
plt.show()