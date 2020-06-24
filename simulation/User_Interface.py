from Interface_3_0 import Ui_MainWindow
from Path_planer import Ui_Path_window
from Camera import Ui_MainCamera
from PyQt5 import QtWidgets, QtCore
import PyQt5.QtGui as QtGui
from PyQt5.QtCore import Qt, QRegExp
import sys
import math
from PyQt5.QtNetwork import QUdpSocket, QHostAddress
import struct
import numpy as np
import time, subprocess

comands = {'start system': 1, 'send_path': 2, 'send_GPS': 3,
     'send_accs': 4, 'send_gyro': 5, 'send_video': 6,
     'send_target_pos': 7, 'send_front_sens': 8, 'send_right_sens': 9,
     'send_left_sens': 10, 'send_orientation_drone': 11, 'send_orientation_target': 12,
     'send_U': 13, 'position drone': 16, 'simtime': 17, 'speed': 14}


class mywindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(mywindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.actionPath_planer.triggered.connect(self.OpenPathWin)
        self.ui.Camera.triggered.connect(self.OpenCamera)
        self.path_ui = PathWin()
        self.Camera_ui = CameraWin()
        self.ui.stop_sim.clicked.connect(self.stop_sim)
        self.ui.start_sim.clicked.connect(self.start_sim)
        self.ui.pause_sim.clicked.connect(self.pause_sim)
        self.ui.Reset_sim.clicked.connect(self.ResetSim)
        self.ui.SpinBox_kp_1.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_ki_1.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kd_1.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_speed_1.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kp_2.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_ki_2.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kd_2.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kp_3.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_ki_3.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kd_3.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kp_4.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_ki_4.valueChanged.connect(self.Change_PID_set)
        self.ui.SpinBox_kd_4.valueChanged.connect(self.Change_PID_set)
        self.ui.Apply_1.clicked.connect(self.SendPid)
        self.ui.Apply_2.clicked.connect(self.SendPid)
        self.ui.Apply_3.clicked.connect(self.SendPid)
        self.ui.Apply_4.clicked.connect(self.SendPid)
        self.ui.Reset_1.clicked.connect(self.ResetPid)
        self.ui.Reset_2.clicked.connect(self.ResetPid)
        self.ui.Reset_3.clicked.connect(self.ResetPid)
        self.ui.Reset_4.clicked.connect(self.ResetPid)
        self.ui.Apply_sim_param.clicked.connect(self.ApplySimParams)
        self.ui.Reset_sim_params.clicked.connect(self.ResetSimParams)



        self.local_ip = QHostAddress("127.0.0.1")
        self.port = 735
        self.udp_socket = QUdpSocket(self)
        self.udp_socket.bind(self.local_ip, self.port)
        self.data = b'0'
        self.threadpool = ReadThread(self.data, self.udp_socket)
        self.threadpool.data_signal.connect(self.Change_sim_info)
        self.threadpool.start()

        self.time = 0
        self.manager_port = 703
        self.kd_1, self.ki_1, self.kp_1, self.speed_1 = [0 for x in range(4)]
        self.kd_2, self.ki_2, self.kp_2, self.speed_2 = [0 for x in range(4)]
        self.kd_3, self.ki_3, self.kp_3 = [0 for x in range(3)]
        self.kd_4, self.ki_4, self.kp_4 = [0 for x in range(3)]

        self.path_ui.ui.create_path.clicked.connect(self.SendField)

    def ApplySimParams(self):
        pass

    def ResetSimParams(self):
        pass

    def ResetPid(self):
        sender = self.sender().objectName()
        if sender == 'Reset_1':
            self.ui.SpinBox_kp_1.setValue(0.0)
            self.ui.SpinBox_ki_1.setValue(0.0)
            self.ui.SpinBox_kd_1.setValue(0.0)
            self.ui.SpinBox_speed_1.setValue(0.0)
        elif sender == 'Reset_2':
            self.ui.SpinBox_kp_2.setValue(0.0)
            self.ui.SpinBox_ki_2.setValue(0.0)
            self.ui.SpinBox_kd_2.setValue(0.0)
            self.ui.SpinBox_speed_2.setValue(0.0)
        elif sender == 'Reset_3':
            self.ui.SpinBox_kp_3.setValue(0.0)
            self.ui.SpinBox_ki_3.setValue(0.0)
            self.ui.SpinBox_kd_3.setValue(0.0)
        elif sender == 'Reset_4':
            self.ui.SpinBox_kp_4.setValue(0.0)
            self.ui.SpinBox_ki_4.setValue(0.0)
            self.ui.SpinBox_kd_4.setValue(0.0)

    def SendPid(self):
        sender = self.sender().objectName()
        if sender == 'Apply_1':
            code = 19
            format = '>h4f'
            massage = struct.pack(format, code, self.kp_1, self.ki_1, self.kd_1, self.speed_1)
            self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)
        if sender == 'Apply_2':
            code = 20
            format = '>h4f'
            massage = struct.pack(format, code, self.kp_2, self.ki_2, self.kd_2, self.speed_2)
            self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)
        if sender == 'Apply_3':
            code = 21
            format = '>h3f'
            massage = struct.pack(format, code, self.kp_3, self.ki_3, self.kd_3)
            self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)
        if sender == 'Apply_4':
            code = 22
            format = '>h3f'
            massage = struct.pack(format, code, self.kp_4, self.ki_4, self.kd_4)
            self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)

    def SendField(self):
        data = np.array(self.path_ui.clicked_histoty).flatten()
        code = 15
        format = '>{}h'.format(data.size + 1)
        massage = struct.pack(format, code, *data)
        self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)

    def OpenPathWin(self):
        self.path_ui.show()

    def OpenCamera(self):
        self.Camera_ui.show()

    def Change_PID_set(self, value):
        sender = self.sender().objectName()
        if sender == 'SpinBox_kd_1':
            self.kd_1 = value
        elif sender == 'SpinBox_ki_1':
            self.ki_1 = value
        elif sender == 'SpinBox_kp_1':
            self.kp_1 = value
        elif sender == 'SpinBox_speed_1':
            self.speed_1 = value

        elif sender == 'SpinBox_kd_2':
            self.kd_2 = value
        elif sender == 'SpinBox_ki_2':
            self.ki_2 = value
        elif sender == 'SpinBox_kp_2':
            self.kp_2 = value

        elif sender == 'SpinBox_kd_3':
            self.kd_3 = value
        elif sender == 'SpinBox_ki_3':
            self.ki_3 = value
        elif sender == 'SpinBox_kp_3':
            self.kp_3 = value

        elif sender == 'SpinBox_kd_4':
            self.kd_4 = value
        elif sender == 'SpinBox_ki_4':
            self.ki_4 = value
        elif sender == 'SpinBox_kp_4':
            self.kp_4 = value

    def ResetSim(self):
        arg = sys.executable + r' C:\Users\Denis\Desktop\CoursProject\simulation\Manager.py'
        self.p = subprocess.Popen(arg)

    def pause_sim(self):
        massage = struct.pack('>h', 17)
        self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)
        self.ui.connection_text.setText('Simulation paused')
        if self.ui.start_sim.isChecked():
            self.ui.start_sim.toggle()

    def start_sim(self):
        massage = struct.pack('>h', 1)
        self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port )
        self.time = time.time()

    def stop_sim(self):
        massage = struct.pack('>h', 18)
        self.udp_socket.writeDatagram(massage, self.local_ip, self.manager_port)
        self.ui.connection_text.setText('Simulation stoped')
        self.p.kill()
        if self.ui.start_sim.isChecked():
            self.ui.start_sim.toggle()



    def Change_sim_info(self, data):
        code = int.from_bytes(data[0:2], "big")
        print(code)
        if code == 3:
            gps_data = list(struct.unpack('>fff', data[2::]))
            self.ui.label_50.setText("{:.3f}".format(gps_data[0]))
            self.ui.label_51.setText("{:.3f}".format(gps_data[1]))
            self.ui.label_52.setText("{:.3f}".format(gps_data[2]))
        elif code == 11:
            orientation_drone = list(struct.unpack('>fff', data[2::]))
            self.ui.label_53.setText(orientation_drone[3])
            self.ui.label_54.setText(orientation_drone[4])
            self.ui.label_55.setText(orientation_drone[5])
        elif code == 17:
            simtime = list(struct.unpack('>f', data[2::]))

        elif code == 14:
            speed = list(struct.unpack('>f', data[2::]))

        elif code == 1:
            self.ui.connection_text.setText('Successfully connected')
        elif code == 6:
            res_1 = struct.unpack('>h', data[2:4])
            res_2 = struct.unpack('>h', data[4:6])
            format = '>' + str(res_1[0] * res_2[0] * 3) + 'h'  # code res, data
            massage = struct.unpack(format, data[6::])
            print(len(massage))

        cur_time = time.time()
        if cur_time - self.time > 1:
            self.ui.sim_time_show.display('{:.2f}'.format(cur_time - self.time))
            self.time = cur_time
        self.update()

class CameraWin(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraWin, self).__init__()
        self.ui = Ui_MainCamera()
        self.ui.setupUi(self)
        self.canvas_1 = QtGui.QPixmap(256, 256)
        self.canvas_2 = QtGui.QPixmap(256, 256)
        self.ui.Raw_camera.setPixmap(self.canvas_1)
        self.ui.filtered_camera.setPixmap(self.canvas_2)

class PathWin(QtWidgets.QMainWindow):
    def __init__(self):
        super(PathWin, self).__init__()
        self.ui = Ui_Path_window()
        self.ui.setupUi(self)
        self.canvas = QtGui.QPixmap(410, 340)
        self.canvas.fill(Qt.white)
        self.ui.planer_field.setPixmap(self.canvas)
        self.DrwaField()
        self.ui.create_path.clicked.connect(self.DrawPath)
        self.ui.reset_path.clicked.connect(self.ResetPath)
        self.ui.auto_planer.clicked.connect(self.AutoPath)
        self.ui.lineEdit.setPlaceholderText('[ x, y]')
        self.ui.lineEdit_2.setPlaceholderText('[ x, y]')
        ex = QRegExp("^\[[0-9]{1,4}\, [0-9]{1,4}\]$")
        validator = QtGui.QRegExpValidator(ex, self)
        self.ui.lineEdit.setValidator(validator)
        self.ui.lineEdit_2.setValidator(validator)
        self.ui.lineEdit.editingFinished.connect(self.GetStart)
        self.ui.lineEdit_2.editingFinished.connect(self.GetRange)
        self.Range = []
        self.Start = []
        self.clicked_histoty = []

    def DrwaField(self):
        painter = QtGui.QPainter(self.ui.planer_field.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(1)
        pen.setColor(QtGui.QColor(138, 193, 236, 60))
        painter.setPen(pen)
        for x in range(0, 410, 41):
            painter.drawLine(
                QtCore.QPoint(x, 0),
                QtCore.QPoint(x, 340)
            )
        for y in range(0, 340, 34):
            painter.drawLine(
                QtCore.QPoint(0, y),
                QtCore.QPoint(410, y)
            )
        self.update()
        painter.end()

    def mousePressEvent(self, e):
        painter = QtGui.QPainter(self.ui.planer_field.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(5)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        x = e.x() - 39
        y = e.y() - 39
        self.clicked_histoty.append([x, y])
        painter.drawPoint(x, y)
        painter.end()
        self.update()

    def DrawPath(self):
        painter = QtGui.QPainter(self.ui.planer_field.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(3)
        pen.setColor(QtGui.QColor(250, 0, 0, 200))
        painter.setPen(pen)
        for i in range(len(self.clicked_histoty)-1):
            painter.drawLine(
                QtCore.QPoint(self.clicked_histoty[i][0], self.clicked_histoty[i][1]),
                QtCore.QPoint(self.clicked_histoty[i+1][0], self.clicked_histoty[i+1][1])
            )
            self.update()
        painter.end()

    def ResetPath(self):
        self.clicked_histoty = []
        painter = QtGui.QPainter(self.ui.planer_field.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(500)
        pen.setColor(QtGui.QColor('white'))
        painter.setPen(pen)
        painter.drawPoint(205, 170)
        pen = QtGui.QPen()
        pen.setWidth(1)
        pen.setColor(QtGui.QColor(138, 193, 236, 60))
        painter.setPen(pen)
        for x in range(0, 410, 41):
            painter.drawLine(
                QtCore.QPoint(x, 0),
                QtCore.QPoint(x, 340)
            )
        for y in range(0, 340, 34):
            painter.drawLine(
                QtCore.QPoint(0, y),
                QtCore.QPoint(410, y)
            )
        self.update()
        painter.end()

    def AutoPath(self):
        wind_size = [410, 340]
        field_size = self.Range
        angle = 60
        self.clicked_histoty.append(self.Start)
        real_step = math.tan(math.radians(60))*5*2
        pix_m_x = field_size[0]/410
        step = round(real_step/pix_m_x)
        painter = QtGui.QPainter(self.ui.planer_field.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(5)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        i = 0
        b = 1
        for x in range(0, 410, step):
            self.clicked_histoty.append([x, 340*i])
            self.clicked_histoty.append([x, 340*b])
            painter.drawPoint(x, 340*i)
            painter.drawPoint(x, 340*b)
            self.update()
            i, b = b, i
        painter.end()

    def GetStart(self):
        s = self.ui.lineEdit.text()
        s = s[1:-1].split(',')
        s = list(map(int, s))
        self.Start = s

    def GetRange(self):
        s = self.ui.lineEdit_2.text()
        s = s[1:-1].split(',')
        s = list(map(int, s))
        self.Range = s


class ReadThread(QtCore.QThread):
    data_signal = QtCore.pyqtSignal(bytes)

    def __init__(self, data, udp_socket):
        super().__init__()

        self.udp_socket = udp_socket
        self.data = data

    def run(self):

        while True:

            while self.udp_socket.hasPendingDatagrams():
                self.data = self.udp_socket.readDatagram(64)
                print(self.data[0])
                self.data_signal.emit(self.data[0])


app = QtWidgets.QApplication([])
application = mywindow()
application.show()
sys.exit(app.exec())