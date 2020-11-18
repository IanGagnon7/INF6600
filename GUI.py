from PyQt5 import QtWidgets, uic, QtCore
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import numpy as np
import random
import socket
import sys

# Server informations
# HOST = '192.168.2.16'  # Server local IP address
# HOST = '127.0.0.1'     # Localhost
HOST = '192.168.56.1'     # Localhost

PORT = 37777        # Port to listen on (non-privileged ports are > 1023)
MAX_TIME_S = 20

# Points that delimited the field
X = [10, 10, 18, 18, 46, 46, 60, 60]
Y = [10, 50, 50, 80, 80, 50, 50, 10]

class DroneServer(QtCore.QObject):
    connectionStatus = QtCore.pyqtSignal(str)
    data = QtCore.pyqtSignal(str)
    def __init__(self):
        super(DroneServer, self).__init__()
        self.conn = None

    @QtCore.pyqtSlot()
    def launch(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.settimeout(MAX_TIME_S)
            s.listen()
            try:
                self.conn, addr = s.accept()
            except socket.timeout:
                self.connectionStatus.emit(f" Timeout after {MAX_TIME_S} sec")
            else:
                self.connectionStatus.emit(f" Connection made with {addr}")

    @QtCore.pyqtSlot()
    def receiveData(self):
        with self.conn:
            while True:
                data = self.conn.recv(1024)
                if not data:
                    print('No data')
                    break
                self.data.emit(data.decode("utf-8"))
                # self.conn.sendall(data)
        


class DroneApp(QtWidgets.QMainWindow):
    launchServer = QtCore.pyqtSignal()
    startReceivingData = QtCore.pyqtSignal()
    def __init__(self):
        # Init Qt object
        super(DroneApp, self).__init__()
        uic.loadUi('GUI.ui', self)
        # Init server and connect signals & slots AFTER moving the object to the thread
        self.server = DroneServer()
        self.serverThread = QtCore.QThread()
        self.server.moveToThread(self.serverThread)
        self.serverThread.start()  
        self.launchServer.connect(self.server.launch)
        self.server.connectionStatus.connect(self.checkConnection)
        self.startReceivingData.connect(self.server.receiveData)
        self.server.data.connect(self.parseData)
        # Init data
        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.battery = 100
        self.memory = 0
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
        self.launchServer.emit()

    @QtCore.pyqtSlot(str)
    def checkConnection(self, connectionStatus):
        self.connectionInfo.setText(connectionStatus)
        if ("Timeout" not in connectionStatus):
            self.startReceivingData.emit()

    @QtCore.pyqtSlot(str)
    def parseData(self, data):
        print(data)
        # Parse data here   

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
        self.xPos += 5
        self.yPos += 5.5
        self.updateXY()
        self.zPos += 0.5
        self.updateZ()
        self.battery -= 5
        self.updateBattery()
        self.memory += 20
        self.updateMemory()

    def updateXY(self):
        self.dronePos.set_data(self.xPos, self.yPos)
        self.map.canvas.draw()

    def updateZ(self):
        self.zSlider.setValue(round(10*self.zPos))
        self.zValue.setText(f"{self.zPos:3.2f} m")

    def updateBattery(self):
        if (self.battery <= 10):
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #cc0000;}")
        elif (self.battery <= 20):
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #dddd00;}")
        else:
            self.batteryProgressBar.setStyleSheet("QProgressBar::chunk {background-color: #00aa00;}")
        self.batteryValue.setText(f"{self.battery:3}%")
        self.batteryProgressBar.setValue(self.battery)

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
