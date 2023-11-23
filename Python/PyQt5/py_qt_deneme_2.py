import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets


from PyQt5.QtWidgets import *
from pyQT_deneme_1 import Ui_MainWindow

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

    xData = []
    yData = []


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
            data_set.system_time = int(self.serial_data[0])
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

    def update_data(self):
        data_set.xData.append(data_set.system_time)
        data_set.yData.append(data_set.accel_z)
        data_set.xData = data_set.xData[-DATA_SIZE:]
        data_set.yData = data_set.yData[-DATA_SIZE:]
        # print(data_set.xData)


class MyMplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=10, height=7, dpi=80, xData=[""], yData=[""]):
        fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        FigureCanvas.__init__(self, fig)
        self.xData = xData
        self.yData = yData
        self.ax.plot(self.xData, self.yData)
        self.ani = animation.FuncAnimation(fig, self.animate, frames=100)

    def animate(self, i):
        plt.cla()
        self.ax.plot(data_set.xData, data_set.yData)
        print(data_set.xData)


#
# class Plotter_App:
#     def __init__(self):
#         self.xData = []
#         self.yData = []
#
#         self.fig, self.ax = plt.subplots(figsize=(5, 5), dpi=80)
#
#         self.ax.plot(self.xData, self.yData)
#         self.ani = animation.FuncAnimation(self.fig, self.animate, frames=100)
#
#     def animate(self, i):
#         plt.cla()
#         self.ax.plot(self.xData, self.yData)
#
#     def update_data(self):
#         self.xData.append(data_set.system_time)
#         self.yData.append(data_set.accel_z)
#         self.xData = self.xData[-DATA_SIZE:]
#         self.yData = self.yData[-DATA_SIZE:]
#         # print(self.xData)
#
#     def get_fig(self):
#         return self.fig


class App:
    def __init__(self):
        self.serial_app = SerialApp()

    def read_serial(self):
        while True:
            self.serial_app.readSerialData()
            self.serial_app.update_data()
            time.sleep(0.001)

    def plot_data(self):
        plt.show()

    def update_label_text(self):
        while True:
            # print("label update")
            pass


class main(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.qtui = Ui_MainWindow()
        self.qtui.setupUi(self)
        self.app = App()

        self.canvas = MyMplCanvas(
            self.qtui.centralwidget,
            width=5,
            height=4,
            dpi=80,
            xData=data_set.xData,
            yData=data_set.yData,
        )
        self.qtui.horizontalLayout_2.addWidget(self.canvas)
        self.canvas.draw()
        #
        # self.qtui.velocity_widget = QtWidgets.QWidget(self.canvas)

        t1 = threading.Thread(target=self.app.read_serial, args=()).start()
        t2 = threading.Thread(target=self.update_label_text, args=()).start()

    def update_label_text(self):
        while True:
            self.qtui.sys_tim_lbl.setText(str(data_set.system_time))
            self.qtui.altitude_label.setText(str(data_set.altitude))
            self.qtui.pressure_label.setText(str(data_set.pressure))
            self.qtui.velocity_label_4.setText(str(data_set.velocity))
            self.qtui.acc_x_label.setText(str(data_set.accel_x))
            self.qtui.acc_y_label.setText(str(data_set.accel_y))
            self.qtui.acc_z_label.setText(str(data_set.accel_z))
            self.qtui.yaw_label_4.setText(str(data_set.yaw))
            self.qtui.pitch_label_5.setText(str(data_set.pitch))
            self.qtui.roll_label_6.setText(str(data_set.roll))
            time.sleep(0.001)


q_app = QApplication([])
pencere = main()
pencere.show()
q_app.exec_()
