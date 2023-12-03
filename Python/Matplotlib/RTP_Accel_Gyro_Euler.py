import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# import numpy as np
import threading

# GETS DATA SET FROM HOLY_TARDIS BY SERIAL PORT

DATA_SIZE = 30


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
        self.yData_accel_x = []
        self.yData_accel_y = []
        self.yData_accel_z = []

        self.yData_gyro_x = []
        self.yData_gyro_y = []
        self.yData_gyro_z = []

        self.yData_euler_yaw = []
        self.yData_euler_pitch = []
        self.yData_euler_roll = []

        self.fig, self.ax = plt.subplots(nrows=3, ncols=3, figsize=(15, 15), dpi=80)

        self.ax[0, 0].plot(self.xData, self.yData_accel_x, label="Accel_X", color="red")
        self.ax[0, 1].plot(self.xData, self.yData_accel_x, label="Accel_Y", color="red")
        self.ax[0, 2].plot(self.xData, self.yData_accel_x, label="Accel_Z", color="red")

        self.ax[1, 0].plot(self.xData, self.yData_gyro_x, label="Gyro_X", color="blue")
        self.ax[1, 1].plot(self.xData, self.yData_gyro_x, label="Gyro_Y", color="blue")
        self.ax[1, 2].plot(self.xData, self.yData_gyro_x, label="Gyro_Z", color="blue")

        self.ax[2, 0].plot(
            self.xData, self.yData_euler_yaw, label="Yaw", color="orange"
        )
        self.ax[2, 1].plot(
            self.xData, self.yData_euler_pitch, label="Pitch", color="orange"
        )
        self.ax[2, 2].plot(
            self.xData, self.yData_euler_roll, label="Roll", color="orange"
        )

        self.ax[0, 0].legend()
        self.ax[0, 1].legend()
        self.ax[0, 2].legend()
        self.ax[1, 0].legend()
        self.ax[1, 1].legend()
        self.ax[1, 2].legend()
        self.ax[2, 0].legend()
        self.ax[2, 1].legend()
        self.ax[2, 2].legend()

        self.ani = animation.FuncAnimation(self.fig, self.animate, frames=1000)

    def animate(self, i):
        self.ax[0, 0].cla()
        self.ax[0, 1].cla()
        self.ax[0, 2].cla()
        self.ax[1, 0].cla()
        self.ax[1, 1].cla()
        self.ax[1, 2].cla()
        self.ax[2, 0].cla()
        self.ax[2, 1].cla()
        self.ax[2, 2].cla()

        self.ax[0, 0].plot(self.xData, self.yData_accel_x, label="Accel_X", color="red")
        self.ax[0, 0].set_ylim(-50, 50)

        self.ax[0, 1].plot(self.xData, self.yData_accel_y, label="Accel_Y", color="red")
        self.ax[0, 1].set_ylim(-50, 50)

        self.ax[0, 2].plot(self.xData, self.yData_accel_z, label="Accel_Z", color="red")
        self.ax[0, 2].set_ylim(-50, 50)

        self.ax[1, 0].plot(self.xData, self.yData_gyro_x, label="Gyro_X", color="blue")
        self.ax[1, 0].set_ylim(-100, 100)

        self.ax[1, 1].plot(self.xData, self.yData_gyro_y, label="Gyro_Y", color="blue")
        self.ax[1, 1].set_ylim(-100, 100)

        self.ax[1, 2].plot(self.xData, self.yData_gyro_z, label="Gyro_Z", color="blue")
        self.ax[1, 2].set_ylim(-100, 100)

        self.ax[2, 0].plot(
            self.xData, self.yData_euler_yaw, label="Yaw", color="orange"
        )
        self.ax[2, 0].set_ylim(-180, 180)

        self.ax[2, 1].plot(
            self.xData, self.yData_euler_pitch, label="Pitch", color="orange"
        )
        self.ax[2, 1].set_ylim(-180, 180)

        self.ax[2, 2].plot(
            self.xData, self.yData_euler_roll, label="Roll", color="orange"
        )
        self.ax[2, 2].set_ylim(0, 360)

    def update_data(self):
        self.xData.append(data_set.system_time)
        self.yData_accel_x.append(data_set.accel_x)
        self.yData_accel_y.append(data_set.accel_y)
        self.yData_accel_z.append(data_set.accel_z)

        self.yData_gyro_x.append(data_set.gyro_x)
        self.yData_gyro_y.append(data_set.gyro_y)
        self.yData_gyro_z.append(data_set.gyro_z)

        self.yData_euler_yaw.append(data_set.yaw)
        self.yData_euler_pitch.append(data_set.pitch)
        self.yData_euler_roll.append(data_set.roll)

        self.xData = self.xData[-DATA_SIZE:]
        self.yData_accel_x = self.yData_accel_x[-DATA_SIZE:]
        self.yData_accel_y = self.yData_accel_y[-DATA_SIZE:]
        self.yData_accel_z = self.yData_accel_z[-DATA_SIZE:]

        self.yData_gyro_x = self.yData_gyro_x[-DATA_SIZE:]
        self.yData_gyro_y = self.yData_gyro_y[-DATA_SIZE:]
        self.yData_gyro_z = self.yData_gyro_z[-DATA_SIZE:]

        self.yData_euler_yaw = self.yData_euler_yaw[-DATA_SIZE:]
        self.yData_euler_pitch = self.yData_euler_pitch[-DATA_SIZE:]
        self.yData_euler_roll = self.yData_euler_roll[-DATA_SIZE:]

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
