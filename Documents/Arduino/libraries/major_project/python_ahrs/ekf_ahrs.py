"""
Python visualization of attitude using Panda3D.
Expects serial lines with 8 comma-separated values:
  columns[1..4] = quaternion (w, x, y, z)
  columns[5..7] = N, E, D (optional, not used here)
Set the COM port and baud as needed; defaults to `COM10` at 2000000.
"""
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q



from panda3d.core import loadPrcFile    #to load .prc file
loadPrcFile("config\config.prc")        #it contains configurational settings

from panda3d.core import NodePath, Loader, Quat
from direct.showbase.ShowBase import ShowBase

import serial


print("\n\n\n\n")


class Game(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.model = self.loader.loadModel("models/misc/rgbCube")
        self.model.reparentTo(render)
        self.model.setPos(0, 100, 0)
        self.model.setScale(20,20,5)

        self.taskMgr.add(self.rotate_model_task, "rotate_model_task")

        self.ser = serial.Serial('COM10', 2000000, timeout=1)
        self.ser.flushInput()
        line = self.ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
        columns = line.split(",")  # split line into columns using comma as delimiter
        

        # wait until we get a valid line
        while len(columns) != 8:
            self.ser.flushInput()
            line = self.ser.readline().decode().rstrip()
            columns = line.split(",")

        if len(columns) == 8:  # make sure there are three columns
            try:
                qw = float(columns[1])
                qx = float(columns[2])
                qy = float(columns[3])
                qz = float(columns[4])


            
            except:
                print("Invalid line: " + line)

        # Set up the initial rotation
        initial_quat = Quat()
        
        initial_quat.setW(qw)
        initial_quat.setX(qz)
        initial_quat.setY(qx)
        initial_quat.setZ(-qy)

        self.model.setQuat(initial_quat)


    def rotate_model_task(self, task):
        self.ser.flushInput()
        line = self.ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
        columns = line.split(",")  # split line into columns using comma as delimiter
        
        global N, E, D

        # wait until we get a valid line
        while len(columns) != 8:
            self.ser.flushInput()
            line = self.ser.readline().decode().rstrip()
            columns = line.split(",")


        if len(columns) == 8:  # make sure there are three columns
            try:
                qw = float(columns[1])
                qx = float(columns[2])
                qy = float(columns[3])
                qz = float(columns[4])

                quat = Quat()
        
                quat.setW(qw)
                quat.setX(qz)
                quat.setY(qx)
                quat.setZ(-qy)

                self.model.setQuat(quat)
            
            except:
                print("Invalid line: " + line)
        
        return task.cont


game = Game()
game.run()