'''manager app'''
import struct
import socket
import subprocess
import sim
import array
from PIL import ImageTk
import PIL.Image as img
import cv2
import numpy as np
import sys, time

commands = {'start system': 1, 'send_path': 2, 'send_GPS': 3,
           'send_accs': 4, 'send_gyro': 5, 'send_video': 6,
           'send_target_pos': 7, 'send_H_sens': 8, 'sensor_ready': 9,
           'path_ready': 10, 'send_orientation_drone': 11, 'send_orientation_target': 12,
           'send_U': 13, 'controll_ready': 14, 'user_path': 15, 'position drone': 16, 'pause': 17,
           'stop_sim': 18, 'Pid_1_set': 19, 'Pid_2_set': 20, 'Pid_3_set': 21,
           'Pid_4_set': 22, 'continue': 23, 'start_video': 24, 'Pid_5_set': 25, 'Pid_6_set': 26, 'velocity': 27}

IP = {'manager': '127.0.0.1', 'obstacle avoidance': '127.0.0.1',
      'Path generator': '127.0.0.1', 'navigation system': '127.0.0.1',
      'object detection': '127.0.0.1', 'inteface': '127.0.0.1',
      'stabilization': '127.0.0.1', 'sensor module': '127.0.0.1'}

ports = {'manager': 703, 'obstacle avoidance': 708,
         'Path generator': 732, 'navigation system': 733,
         'object detection': 734, 'inteface': 735,
         'stabilization': 736, 'sensor module': 746}




class initialization:
    def __init__(self, IP, ports):
        self.IP = IP
        self.ports = ports
        self.local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.local_socket.bind((self.IP, self.ports))


    def get_params(self):
        return self.IP, self.ports


class massage(initialization):
    def __init__(self, IP, ports):
        super().__init__(IP, ports)

    def get_massage(self):
        massage = self.local_socket.recv(512)
        code = int.from_bytes(massage[0:2], "big")

        if code == 2 or code == 3 or code == 4 or code == 5 or code == 7 or code == 16 \
                or code == 11 or code == 12 or code == 22 \
                or code == 25 or code == 26 or code == 27 or code == 20 or code == 21:
            massage = list(struct.unpack('>3f', massage[2::]))
            return [code, (*massage)]
        elif code == 8:
            massage = list(struct.unpack('>?6f', massage[2::]))
            return [code, (*massage)]
        elif code == 1 or code == 17 or code == 18 or code == 23 or code == 9 or code == 10 or code == 14 or code == 24:
            return [code]
        elif code == 13 or code == 19:
            massage = list(struct.unpack('>4f',  massage[2::]))
            return [code, (*massage)]
        elif code == 15:
            size = int.from_bytes(massage[2:4], "big")
            print(f'size = {size}')
            massage = list(struct.unpack(f'>{size}h', massage[4::]))
            return [code, (*massage)]

    def send_massage(self, code, data, IP, port, res = None):
        if code == 2 or code == 3 or code == 4 or code == 5 or code == 7 \
                or code == 16 or code == 11 or code == 12 or code == 22\
                or code == 25 or code == 26 or code == 27 or code == 20 or code == 21:
            format = '>h3f'
            massage = struct.pack(format, code, *data)
            self.local_socket.sendto(massage, (IP, port))
        elif code == 8:
            format = '>h?6f'
            massage = struct.pack(format, code, data[0], *data[1], *data[2])
            self.local_socket.sendto(massage, (IP, port))
        elif code == 1 or code == 17 or code == 18 or code == 23 or code == 9 or code == 10 or code == 14 or code == 24:
            format = '>h'
            massage = struct.pack(format, code)
            self.local_socket.sendto(massage, (IP, port))
        elif code == 13 or code == 19:
            print(data)
            format = '>h4f'
            massage = struct.pack(format, code, *data)
            self.local_socket.sendto(massage, (IP, port))
        else:
            return 'Unexpected command'

reset_sim, stop_sim, start_sim = [False for x in range(3)]
massage_manager = massage(IP['manager'], ports['manager'])


# посылвем всем модулям команду о готовности к работе do with outstanding method
sensor_ready = False

while not sensor_ready:
    if not sensor_ready:
        massage_manager.send_massage(commands['start system'], None, IP['sensor module'], ports['sensor module'])
    code = massage_manager.get_massage()
    if code[0] == 9:
        sensor_ready = True



start_sim = True
while True:

    while start_sim:
        input_data = massage_manager.get_massage()

        if input_data[0] == 24:
            massage_manager.send_massage(commands['start_video'], 1, IP['sensor module'], ports['sensor module'])
        elif input_data[0] == 3 or input_data[0] == 11:
            massage_manager.send_massage(input_data[0], input_data[1::], IP['inteface'], ports['inteface'])
        elif input_data[0] == 20 or input_data[0] == 21 or input_data[0] == 22 or \
                input_data[0] == 25 or input_data[0] == 26 or input_data[0] == 19:
            massage_manager.send_massage(input_data[0], input_data[1::], IP['sensor module'], ports['sensor module'])


    input_data = massage_manager.get_massage()
    if input_data[0] == 1:
        start_sim = True
        massage_manager.send_massage(commands['continue'], 1, IP['sensor module'], ports['sensor module'])
