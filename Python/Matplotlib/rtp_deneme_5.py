import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# import numpy as np
import threading

# GETS DATA SET FROM HOLY_TARDIS BY SERIAL PORT

DATA_SIZE = 200


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
    def __init__(self):
        try:
            self.ser = serial.Serial("COM6", 115200)
        except:
            print("Port can't opened!!!")
            sys.exit()

    def readSerialData(self):
        try:
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

            # print(self.serial_data)
        except:
            print("Data fail!")


class Plotter_App:
    def __init__(self):
        self.xData = []
        self.yData = []

        self.fig, self.ax = plt.subplots(figsize=(5, 5), dpi=80)

        self.ax.plot(self.xData, self.yData)
        self.ani = animation.FuncAnimation(self.fig, self.animate, frames=100)

    def animate(self, i):
        plt.cla()
        self.ax.plot(self.xData, self.yData)

    def update_data(self):
        self.xData.append(data_set.system_time)
        self.yData.append(data_set.accel_z)
        self.xData = self.xData[-DATA_SIZE:]
        self.yData = self.yData[-DATA_SIZE:]
        # print(self.xData)


class App:
    def __init__(self):
        self.serial_app = SerialApp()
        self.plotter_app = Plotter_App()

    def read_serial(self):
        while True:
            self.serial_app.readSerialData()
            self.plotter_app.update_data()

    def plot_data(self):
        plt.show()


app = App()

t1 = threading.Thread(target=app.read_serial, args=()).start()
app.plot_data()
