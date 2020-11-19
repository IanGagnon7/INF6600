from PyQt5 import QtWidgets, uic, QtCore
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import numpy as np
import random
import socket
import sys

# Server informations
HOST = '192.168.56.128'  # Server local IP address
# HOST = '127.0.0.1'  # Server local IP address
PORT = 37777        # Port to listen on (non-privileged ports are > 1023)
MAX_TIME_S = 5
SIMU_END = '\x03'
MSGLEN = 12

# Points that delimited the field
X = [10, 10, 18, 18, 46, 46, 60, 60]
Y = [10, 50, 50, 80, 80, 50, 50, 10]

class DroneClient(QtCore.QObject):
    connectionStatus = QtCore.pyqtSignal(str)
    dataStr = QtCore.pyqtSignal(str)
    def __init__(self):
        super(DroneClient, self).__init__()
        self.s = None

    @QtCore.pyqtSlot()
    def connect_to_server(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.s.connect((HOST, PORT))
        except (TimeoutError, ConnectionRefusedError):
            self.connectionStatus.emit(f" Connection FAILED.")
            self.s.close()
        else:
            self.s.sendall(b'Start')
            self.s.settimeout(MAX_TIME_S)
            self.connectionStatus.emit(f" Connection ESTABLISHED!")

    @QtCore.pyqtSlot()
    def receiveData(self):
        last_car = None
        data = ""
        while last_car != SIMU_END:
            try:
                chunk = self.s.recv(min(MSGLEN - len(data), 1024)).decode("utf-8")
            except socket.timeout:
                self.connectionStatus.emit(f" Connection LOST!")
                break
            data += chunk
            last_car = data[-1]
            if len(data) >= MSGLEN:
                self.dataStr.emit(data)
                data = ""
        else:
            self.connectionStatus.emit(f" Connection ENDED by the drone!")
        self.s.close()

class DroneApp(QtWidgets.QMainWindow):
    connectClient = QtCore.pyqtSignal()
    startReceivingData = QtCore.pyqtSignal()
    def __init__(self):
        # Init Qt object
        super(DroneApp, self).__init__()
        uic.loadUi('GUI.ui', self)
        # Init server and connect signals & slots AFTER moving the object to the thread
        self.client = DroneClient()
        self.clientThread = QtCore.QThread()
        self.client.moveToThread(self.clientThread)
        self.clientThread.start()  
        self.connectClient.connect(self.client.connect_to_server)
        self.client.connectionStatus.connect(self.checkConnection)
        self.startReceivingData.connect(self.client.receiveData)
        self.client.dataStr.connect(self.parseData)
        # Init data
        self.memory = 0
        self.battery = 100
        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        
        # Init UI
        self.initUI()

    def initUI(self):
        self.testPushButton.clicked.connect(self.updateAll)
        self.connectPushButton.clicked.connect(self.connectDrone)
        self.createMap()
        self.updateBattery()
        self.updateMemory()
        self.updateZ()
        self.connectionInfo.setStyleSheet("background-color: black; color:white")
        # self.toggleButton.clicked.connect(self.clicked)

    def connectDrone(self):
        self.connectionInfo.setText(" Waiting for a connection...")
        self.connectClient.emit()

    @QtCore.pyqtSlot(str)
    def checkConnection(self, connectionStatus):
        self.connectionInfo.setText(connectionStatus)
        if 'ESTABLISHED' in connectionStatus:
            self.startReceivingData.emit()

    @QtCore.pyqtSlot(str)
    def parseData(self, dataStr):
        dataArray = [float(data) for data in dataStr.strip().split(',')]
        print(dataArray)
        self.memory = round(dataArray[0]/10e9)
        self.battery = round(dataArray[1], 1)
        self.xPos = round(dataArray[2], 2)
        self.yPos = round(dataArray[3], 2)
        self.zPos = round(dataArray[4], 2)
        self.updateAll()

    def createMap(self):
        self.map.canvas.axes.clear()
        self.map.canvas.axes.grid()
        self.map.canvas.axes.plot(0, 0, color='red', marker='o', markersize=12)
        X.append(X[0]) # Add the last line to close the field
        Y.append(Y[0])
        self.map.canvas.axes.plot(X, Y, color='royalblue', marker='o', linewidth=2, markersize=7, markerfacecolor='gray')
        self.map.canvas.axes.fill(X, Y, 'lightcyan')
        self.map.canvas.axes.set_title('Live position of drone')

        self.dronePos, = self.map.canvas.axes.plot(self.xPos, self.yPos, color='black', marker='o', markersize=10)
        self.map.canvas.draw()

    def updateAll(self):
        self.updateXY()
        self.updateZ()
        self.updateBattery()
        self.updateMemory()

    def updateXY(self):
        self.dronePos.set_data(self.xPos, self.yPos)
        self.map.canvas.draw()

    def updateZ(self):
        self.zSlider.setValue(round(10*self.zPos))
        self.zValue.setText(f"{self.zPos:3} m")

    def updateBattery(self):
        if (self.battery <= 10):
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #cc0000;}")
        elif (self.battery <= 20):
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #dddd00;}")
        else:
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #00aa00;}")
        self.batteryValue.setText(f"{self.battery:4}%")
        self.batteryProgressBar.setValue(10*self.battery)

    def updateMemory(self):
        if (self.memory >= 500):
            self.memoryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #cc0000;}")
        elif (self.memory >= 400):
            self.memoryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #dddd00;}")
        else:
            self.memoryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #00aa00;}")
        self.memoryValue.setText(f"{self.memory:3} MB")
        self.memoryProgressBar.setValue(self.memory)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = DroneApp()
    win.show()
    sys.exit(app.exec_())
