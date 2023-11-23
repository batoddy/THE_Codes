import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import time


class dataSetClass:
    system_time = 0
    altitude = 0
    pressure = 0
    velocity = 0
    accel_x = 0
    accel_y = 0
    accel_z = 0
    gyro_x = 0
    gyro_y = 0
    gyro_z = 0
    yaw = 0
    pitch = 0
    roll = 0


data_set = dataSetClass()


class SerialApp:
    # def __init__(self):
    #     self.system_time = 0
    #     self.altitude = 0
    #     self.pressure = 0
    #     self.velocity = 0
    #     self.accel_x = 0
    #     self.accel_y = 0
    #     self.accel_z = 0
    #     self.gyro_x = 0
    #     self.gyro_y = 0
    #     self.gyro_z = 0
    #     self.yaw = 0
    #     self.pitch = 0
    #     self.roll = 0

    def openSerial(self):
        try:
            self.ser = serial.Serial("COM6", 115200)
            return self.ser
        except:
            print("Port can't opened!!")
            sys.exit()

    def readSerialData(self):
        # try:
        try:
            self.ser = self.openSerial()
            self.serial_data = self.ser.readline()[:-1].decode("latin1").split(",")
            data_set.system_time = float(self.serial_data[0])
            data_set.altitude = float(self.serial_data[1])
            data_set.pressure = float(self.serial_data[2])
            data_set.velocity = float(self.serial_data[3])
            data_set.accel_x = float(self.serial_data[4])
            data_set.accel_y = float(self.serial_data[5])
            data_set.accel_z = float(self.serial_data[6])
            data_set.gyro_x = float(self.serial_data[7])
            data_set.gyro_y = float(self.serial_data[8])
            data_set.gyro_z = float(self.serial_data[9])
            data_set.yaw = float(self.serial_data[10])
            data_set.pitch = float(self.serial_data[11])
            data_set.roll = float(self.serial_data[12])
        except:
            print("Data can't read clearly!!")
        # print(
        #     f"Time: {self.system_time}, Altitude: {self.altitude}, Pressure: {self.pressure}, Velocity: {self.velocity}, Accel_X: {self.accel_x}, Accel_Y: {self.accel_y}, Accel_Z: {self.accel_z}, Gyro_X: {self.gyro_x}, Gyro_Y: {self.gyro_y}, Gyro_Z: {self.gyro_z}, Yaw: {self.yaw}, Pitch: {self.pitch}, Roll: {self.roll}"
        # )

    # except:
    #     print("Can't read data!!!")


class App:
    def real_time_plot(self):
        self.serial_app = SerialApp()
        self.serial_app.readSerialData()
        print(data_set.system_time, data_set.pressure)


app = App()
while 1:
    app.real_time_plot()
