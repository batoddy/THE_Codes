import time
import serial
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation


#with open('data.csv', 'a') as f:
        #f.write(
        #f'{"Tick"},{"pkg_no"},{"flight_state"},{"stabilzation_flag"},{"Altitude"},{"Pressure"},{"Velocity"},{"Yaw"},{"Pitch"},{"Roll"},{"Abs_Accel"},{"Accel_X"},{"Accel_Y"},{"Accel_Z"}\n')
        
def openSerial():
    try:
        ser = serial.Serial('COM5', 115200)
        # ser = serial.Serial('COM7', 115200)

        return ser
    except:
        print("Could not open serial port")
        sys.exit(1)

def getFromSerial():
    ser = openSerial()
    
    # while True:
    try: 
        serialData = ser.readline()[:-1].decode('latin1').split(",")

        #alt = "{:.2f}".format(serialData[4])
        #velo ="{:.2f}".format(serialData[5])
        #accel_z = "{:.2f}".format(serialData[6])
        #yaw = "{:.2f}".format(serialData[7])
        #pitch = "{:.2f}".format(serialData[8])
        #roll = "{:.2f}".format(serialData[9])
        #rocket_max = "{:.2f}".format(serialData[10])
        #payload_max = "{:.2f}".format(serialData[11])

        print(f'T: {serialData[0]} Pkg: {serialData[1]} FS: {serialData[2]} Alt: {serialData[4]} Velo: {serialData[5]} A_Z: {serialData[6]} Y: {serialData[7]} P: {serialData[8]} R: {serialData[9]} DC: {serialData[15]} UTC: {serialData[10]}L_Lat: {serialData[11]} L_Lng: {serialData[12]} Tmp: {serialData[21]} Rkt_mx: {serialData[19]} Pld_mx: {serialData[21]} ')
         #save data to file
    except: 
        print("Data read err!!!");


    with open('data.csv', 'a') as f:
        try:
            f.write(
                f'{serialData[0]},{serialData[1]},{serialData[2]},{serialData[3]},{serialData[4]},{serialData[5]},{serialData[6]},{serialData[7]},{serialData[8]},{serialData[9]},{serialData[10]},{serialData[11]},{serialData[12]},{serialData[13]},{serialData[14]},{serialData[15]},{serialData[16]},{serialData[17]},{serialData[18]},{serialData[19]},{serialData[20]},{serialData[21]},{serialData[22]},{serialData[23]}\n')
                # 
        except:
            print("Data save err!!!")
openSerial()
while(1):
    getFromSerial()

