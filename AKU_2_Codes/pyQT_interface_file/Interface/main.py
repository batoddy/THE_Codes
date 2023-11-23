import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib import style
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

import threading
import time
import random as rnd

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap

from PyQt5.QtWidgets import *
from aku_ui import Ui_MainWindow


DATA_SIZE = 100


class dataSetClass:
    # flight data
    system_time = 0
    altitude = 0
    velocity = 0
    pressure = 0
    temprature = 0
    accel_x = 0
    accel_y = 0
    accel_z = 0
    gyro_x = 0
    gyro_y = 0
    gyro_z = 0
    yaw = 0
    pitch = 0
    roll = 0
    displacement_x = 0
    displacement_y = 0
    displacement_z = 0
    magno_x = 0
    magno_y = 0
    magno_z = 0
    servo1 = 0
    servo2 = 0

    dumy_displacement_x = 0
    dumy_displacement_y = 0
    dumy_displacement_z = 0

    # flight control
    flight_state = "LIFT_OFF"
    recovery_signal = "HIGH"
    stabilization_flag = "STABLE"

    # gps data
    utc_time = 0
    latitude = 41.027306
    longtitude = 28.884898

    # telemetry data
    baundrate = "440.0 Mhz"
    frequency = "512 Hz"
    pkg_no = 100
    latency = 150
    bps = 3452

    xData = []
    yData_altitude = []
    yData_accel_z = []
    yData_latency = []

    xData_displacement = []
    yData_displacement = []
    zData_displacement = []


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
            # dummy time
            data_set.latency = data_set.latency + rnd.randint(-1, 1)
            data_set.dumy_displacement_x = data_set.dumy_displacement_x + rnd.randint(
                -1, 1
            )
            data_set.dumy_displacement_y = data_set.dumy_displacement_y + rnd.randint(
                -1, 1
            )
            data_set.dumy_displacement_z = data_set.dumy_displacement_z + rnd.randint(
                0, 100
            )
            data_set.utc_time = time.strftime("%H:%M:%S", time.localtime())

            # print(self.serial_data)
        except:
            print("Data fail!")

    def update_data(self):
        data_set.xData.append(data_set.system_time)
        data_set.yData_altitude.append(data_set.altitude)
        data_set.yData_accel_z.append(data_set.accel_z)
        data_set.yData_latency.append(data_set.latency)

        data_set.xData = data_set.xData[-DATA_SIZE:]
        data_set.yData_altitude = data_set.yData_altitude[-DATA_SIZE:]
        data_set.yData_accel_z = data_set.yData_accel_z[-DATA_SIZE:]
        data_set.yData_latency = data_set.yData_latency[-DATA_SIZE:]

        data_set.xData_displacement.append(data_set.dumy_displacement_x)
        data_set.yData_displacement.append(data_set.dumy_displacement_y)
        data_set.zData_displacement.append(data_set.dumy_displacement_z)

        data_set.xData_displacement = data_set.xData_displacement[-DATA_SIZE:]
        data_set.yData_displacement = data_set.yData_displacement[-DATA_SIZE:]
        data_set.zData_displacement = data_set.zData_displacement[-DATA_SIZE:]

        # print(data_set.xData)


class MyMplCanvas(FigureCanvas):
    def __init__(
        self,
        parent=None,
        width=10,
        height=7,
        dpi=80,
        xData=[""],
        yData=[""],
        zData=[""],
        plot_data="",
        dimention="2d",
    ):
        # plt.style.use("dark_background")
        # #d1d1d1
        self.xData = xData
        self.yData = yData
        self.zData = zData
        self.plot_data = plot_data
        self.dimention = dimention

        if self.dimention == "2d":
            fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
            # fig.set_facecolor("#d1d1d1")
            fig.patch.set_alpha(0)
            FigureCanvas.__init__(self, fig)

            self.plot_func(self.xData, self.yData)

            print("2d dimention plot")

            self.ani = animation.FuncAnimation(fig, self.animate2d, frames=1000)
        elif self.dimention == "3d":
            fig = plt.figure(figsize=(width, height), dpi=dpi)
            self.ax = plt.axes(projection="3d")
            # fig.set_facecolor("#d1d1d1")
            fig.patch.set_alpha(0)
            FigureCanvas.__init__(self, fig)
            print("3d dimention plot")

            self.plot_func_3d(self.xData, self.yData, self.zData)

            self.ani = animation.FuncAnimation(fig, self.animate3d, frames=100)

    def plot_func(self, xData=[], yData=[]):
        self.ax.grid(color="gray", linewidth="0.4")
        self.ax.set_facecolor("black")
        self.ax.tick_params(axis="x", color="white", labelcolor="white")
        self.ax.tick_params(axis="y", color="white", labelcolor="white")

        # Add a frame around the plot area
        self.ax.spines["top"].set_visible(True)
        self.ax.spines["right"].set_visible(True)
        self.ax.spines["bottom"].set_visible(True)
        self.ax.spines["left"].set_visible(True)

        # Set the color of the frame
        frame_color = "white"
        self.ax.spines["top"].set_color(frame_color)
        self.ax.spines["right"].set_color(frame_color)
        self.ax.spines["bottom"].set_color(frame_color)
        self.ax.spines["left"].set_color(frame_color)

        # Set the linewidth of the frame
        frame_linewidth = 1.5
        self.ax.spines["top"].set_linewidth(frame_linewidth)
        self.ax.spines["right"].set_linewidth(frame_linewidth)
        self.ax.spines["bottom"].set_linewidth(frame_linewidth)

        self.ax.spines["left"].set_linewidth(frame_linewidth)
        self.ax.plot(xData, yData, color="orange")

    def plot_func_3d(self, xData=[], yData=[], zData=[]):
        self.ax.grid(color="gray", linewidth="0.1", alpha=0.1)

        self.ax.set_facecolor("#0a0a0a")

        self.ax.tick_params(axis="x", color="white", labelcolor="white")
        self.ax.tick_params(axis="y", color="white", labelcolor="white")
        self.ax.tick_params(axis="z", color="white", labelcolor="white")

        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False

        self.ax.plot3D(xData, yData, zData, color="orange")
        print("plot_func_3d")

    def animate2d(self, i):
        match (self.plot_data):
            case "altitude":
                self.ax.cla()
                self.ax.set_ylim(-5, 200)
                self.ax.set_title("Altitude (m)", color="white")
                self.plot_func(data_set.xData, data_set.yData_altitude)
            # print(data_set.yData_altitude)

            case "accel_z":
                self.ax.cla()
                self.ax.set_ylim(-20, 50)
                self.ax.set_title("Acceleration (m/s2)", color="white")
                self.plot_func(data_set.xData, data_set.yData_accel_z)

            case "latency":
                self.ax.cla()
                self.ax.set_ylim(10, 250)
                self.ax.set_title("Latency (ms)", color="white")
                self.plot_func(data_set.xData, data_set.yData_latency)

            case "xz_displacement":
                self.ax.cla()
                self.ax.set_xlim(-50, 50)
                self.ax.set_title("XZ Displacement", color="white")
                self.plot_func(data_set.xData_displacement, data_set.zData_displacement)

            case "yz_displacement":
                self.ax.cla()
                self.ax.set_xlim(-50, 50)
                self.ax.set_title("YZ Displacement", color="white")
                self.plot_func(data_set.yData_displacement, data_set.zData_displacement)

        # if self.plot_data == "altitude":
        #    self.ax.cla()
        #    self.ax.set_ylim(-5, 200)
        #    self.ax.set_title("Altitude (m)", color="white")
        #    self.plot_func(data_set.xData, data_set.yData_altitude)
        #    # print(data_set.yData_altitude)

        # elif self.plot_data == "accel_z":
        #    self.ax.cla()
        #    self.ax.set_ylim(-20, 50)
        #    self.ax.set_title("Acceleration (m/s2)", color="white")
        #    self.plot_func(data_set.xData, data_set.yData_accel_z)

        # elif self.plot_data == "latency":
        #    self.ax.cla()
        #    self.ax.set_ylim(10, 300)
        #    self.ax.set_title("Latency (ms)", color="white")
        #    self.plot_func(data_set.xData, data_set.yData_latency)

    def animate3d(self, i):
        self.ax.cla()
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_zlim(-10, 3000)
        self.plot_func_3d(
            data_set.xData_displacement,
            data_set.yData_displacement,
            data_set.zData_displacement,
        )
        print("animate_3d")

        # print(data_set.yData_altitude)


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
        self.add_images_manually()

        self.canvas_altitude = MyMplCanvas(
            self.qtui.centralwidget,
            width=5,
            height=4,
            dpi=80,
            xData=data_set.xData,
            yData=data_set.yData_altitude,
            plot_data="altitude",
            dimention="2d",
        )

        self.canvas_accel_z = MyMplCanvas(
            self.qtui.centralwidget,
            width=5,
            height=4,
            dpi=80,
            xData=data_set.xData,
            yData=data_set.yData_accel_z,
            plot_data="accel_z",
            dimention="2d",
        )

        self.canvas_latency = MyMplCanvas(
            self.qtui.centralwidget,
            width=5,
            height=2,
            dpi=80,
            xData=data_set.xData,
            yData=data_set.yData_latency,
            plot_data="latency",
            dimention="2d",
        )

        self.canvas_displacement_xz = MyMplCanvas(
            self.qtui.centralwidget,
            width=4,
            height=8,
            dpi=100,
            xData=data_set.xData_displacement,
            yData=data_set.zData_displacement,
            plot_data="xz_displacement",
            dimention="2d",
        )

        self.canvas_displacement_yz = MyMplCanvas(
            self.qtui.centralwidget,
            width=4,
            height=8,
            dpi=100,
            xData=data_set.yData_displacement,
            yData=data_set.zData_displacement,
            plot_data="yz_displacement",
            dimention="2d",
        )

        self.canvas_displacement_xyz = MyMplCanvas(
            self.qtui.centralwidget,
            width=30,
            height=30,
            dpi=100,
            xData=data_set.xData_displacement,
            yData=data_set.yData_displacement,
            zData=data_set.zData_displacement,
            plot_data="latency",
            dimention="3d",
        )

        self.qtui.altitude_Vlayout.addWidget(self.canvas_altitude)
        self.qtui.accelz_Vlayout.addWidget(self.canvas_accel_z)
        self.qtui.latency_Vlayout.addWidget(self.canvas_latency)
        self.qtui.xz_verticalLayout.addWidget(self.canvas_displacement_xz)
        self.qtui.yz_verticalLayout.addWidget(self.canvas_displacement_yz)
        self.qtui.dimention3_verticalLayout.addWidget(self.canvas_displacement_xyz)

        self.canvas_altitude.draw()
        self.canvas_accel_z.draw()
        self.canvas_latency.draw()
        self.canvas_displacement_xz.draw()
        self.canvas_displacement_yz.draw()
        self.canvas_displacement_xyz.draw()

        #
        # self.qtui.velocity_widget = QtWidgets.QWidget(self.canvas)

        t1 = threading.Thread(target=self.app.read_serial, args=()).start()
        t2 = threading.Thread(target=self.update_label_text, args=()).start()

    def add_images_manually(self):
        self.yrt_logo_pixmap = QPixmap(
            "C:/Users/batuh/OneDrive/Masaüstü/Yildiz Roket/AKU/AKU-2/Interface/pyQT uygulamalari/pyQT_interface_file/images/yrt_logo.png"
        )
        self.yrt_logo_2_pixmap = QPixmap(
            "C:/Users/batuh/OneDrive/Masaüstü/Yildiz Roket/AKU/AKU-2/Interface/pyQT uygulamalari/pyQT_interface_file/images/yrt_logo_2.png"
        )
        self.aku_patch_pixmap = QPixmap(
            "C:/Users/batuh/OneDrive/Masaüstü/Yildiz Roket/AKU/AKU-2/Interface/pyQT uygulamalari/pyQT_interface_file/images/aku_patch.png"
        )

        self.qtui.yrt_logo_lbl.setPixmap(self.yrt_logo_pixmap.scaled(202, 180))
        self.qtui.yrt_logo_2_lbl.setPixmap(self.yrt_logo_2_pixmap.scaled(648, 180))
        self.qtui.aku_patch_lbl.setPixmap(self.aku_patch_pixmap.scaled(202, 180))

    def update_label_text(self):
        while True:
            self.qtui.system_time_lbl.setText(str(data_set.system_time))
            self.qtui.altitude_lbl.setText(str(data_set.altitude))
            self.qtui.pressure_lbl.setText(str(data_set.pressure))
            self.qtui.velocity_lbl.setText(str(data_set.velocity))
            self.qtui.temprature_lbl.setText(str(data_set.temprature))
            self.qtui.accelz_lbl.setText(str(data_set.accel_x))
            self.qtui.accely_lbl.setText(str(data_set.accel_y))
            self.qtui.accelz_lbl_2.setText(str(data_set.accel_z))
            self.qtui.yaw_lbl.setText(str(data_set.yaw))
            self.qtui.pitch_lbl.setText(str(data_set.pitch))
            self.qtui.roll_lbl.setText(str(data_set.roll))
            self.qtui.displacementx_lbl.setText(str(data_set.displacement_x))
            self.qtui.displacementy_lbl.setText(str(data_set.displacement_y))
            self.qtui.displacementz_lbl.setText(str(data_set.displacement_z))
            self.qtui.magnox_lbl.setText(str(data_set.magno_x))
            self.qtui.magnoy_lbl.setText(str(data_set.magno_y))
            self.qtui.magnoz_lbl.setText(str(data_set.magno_z))
            self.qtui.servo1_lbl.setText(str(data_set.servo1))
            self.qtui.servo2_lbl.setText(str(data_set.servo2))

            self.qtui.flight_state_lbl.setText(str(data_set.flight_state))
            self.qtui.recovery_signal_lbl.setText(str(data_set.recovery_signal))
            self.qtui.label_37.setText(str(data_set.stabilization_flag))

            self.qtui.utc_time_lbl.setText(str(data_set.utc_time))
            self.qtui.latitude_lbl.setText(str(data_set.latitude))
            self.qtui.longtitude_lbl.setText(str(data_set.longtitude))

            self.qtui.frequency_lbl.setText(str(data_set.frequency))
            self.qtui.baundrate_lbl.setText(str(data_set.baundrate))
            self.qtui.pkg_no_lbl.setText(str(data_set.pkg_no))
            self.qtui.latency_lbl.setText(str(data_set.latency))
            self.qtui.bps_lbl.setText(str(data_set.bps))

            time.sleep(0.001)


q_app = QApplication([])
pencere = main()
pencere.show()
q_app.exec_()
