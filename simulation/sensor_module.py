import struct
import socket
import time
import sim
import array, sys
import cv2
from PIL import ImageTk
import PIL.Image as img

import numpy as np

comands = {'start system': 1, 'send_path': 2, 'send_GPS': 3,
           'send_accs': 4, 'send_gyro': 5, 'send_video': 6,
           'send_target_pos': 7, 'send_H_sens': 8, 'sensor_ready': 9,
           'path_ready': 10, 'send_orientation_drone': 11, 'send_orientation_target': 12,
           'send_U': 13, 'controll_ready': 14, 'user_path': 15, 'position drone': 16, 'pause': 17,
           'stop_sim': 18, 'Pid_1_set': 19, 'Pid_2_set': 20, 'Pid_3_set': 21,
           'Pid_4_set': 22, 'continue': 23}

IP = {'manager': '127.0.0.1', 'obstacle avoidance': '127.0.0.1',
      'Path generator': '127.0.0.1', 'navigation system': '127.0.0.1',
      'object detection': '127.0.0.1', 'inteface': '127.0.0.1',
      'stabilization': '127.0.0.1', 'sensor module': '127.0.0.1'}

ports = {'manager': 703, 'obstacle avoidance': 708,
         'Path generator': 732, 'navigation system': 733,
         'object detection': 734, 'inteface': 735,
         'stabilization': 736, 'sensor module': 746}






class initialization:
    def __init__(self, IP, port):
        self.IP = IP
        self.port = port
        self.local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.local_socket.bind((self.IP, self.port))



    def get_params(self):
        return self.IP, self.port


class massage(initialization):
    def __init__(self, IP, ports):
        super().__init__(IP, ports)

    def get_massage(self):
        massage = self.local_socket.recv(128)
        code = int.from_bytes(massage[0:2], "big")
        print(code)
        if code == 2 or code == 3 or code == 4 or code == 5 or code == 7 or code == 16 \
                or code == 11 or code == 12 or code == 21 or code == 22:
            massage = list(struct.unpack('>fff', massage[2::]))
            return code, (*massage)
        elif code == 8:
            massage = list(struct.unpack('>?6f', massage[2::]))
            return code, (*massage)
        elif code == 1 or code == 17 or code == 18 or code == 23 or code == 9 or code == 10 or code == 14:
            return [code]
        elif code == 13 or code == 19 or code == 20:
            massage = list(struct.unpack('>4f',  massage[2::]))
            return code, (*massage)


    def send_massage(self, code, data, IP, port, res = None):
        if data:
            if code == 2 or code == 12 or code == 4 or code == 5 or code == 7 or code == 16 or code == 11 or code == 3:
                format = '>hfff'
                massage = struct.pack(format, code, *data)
                self.local_socket.sendto(massage, (IP, port))
            elif code == 8:
                format = '>h?6f'
                massage = struct.pack(format, code, data[0], *data[1], *data[2])
                self.local_socket.sendto(massage, (IP, port))
            elif code == 1 or code == 9:
                format = '>h'
                massage = struct.pack(format, code)
                self.local_socket.sendto(massage, (IP, port))
            elif code == 6:
                print('send')
                format = '>hhh' + str(res[0]*res[1]*3) + 'h' # code res, data
                data = np.array(data).flatten()
                massage = struct.pack(format, code, res[0], res[1], *data)
                self.local_socket.sendto(massage, (IP, port))
            else:
                return 'Unexpected format'




massage = massage(IP['sensor module'], ports['sensor module'])
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 15000, 5) # Connect to CoppeliaSim
while clientID == -1:
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
    print('Not connected to remote API server')
    if clientID != -1:
        break


start_sim = True

# simxGetPingTime(clientID)

res, drone = sim.simxGetObjectHandle(clientID, 'my_drone', sim.simx_opmode_blocking)
res, target = sim.simxGetObjectHandle(clientID, 'Quadcopter_target', sim.simx_opmode_blocking)
res, camera = sim.simxGetObjectHandle(clientID, 'Main_camera', sim.simx_opmode_oneshot_wait)
res, h_sensor = sim.simxGetObjectHandle(clientID, 'H_sensor', sim.simx_opmode_oneshot_wait)
res, gps = sim.simxGetStringSignal(clientID, 'gps', sim.simx_opmode_streaming)
res, gyro = sim.simxGetStringSignal(clientID, 'gyro', sim.simx_opmode_streaming)
res, accel = sim.simxGetStringSignal(clientID, 'nydavai', sim.simx_opmode_streaming)


res, h_detected, h_point, h_det_obj, h_norm = sim.simxReadProximitySensor(clientID, h_sensor, sim.simx_opmode_streaming)
res, orientation_target = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_streaming)
res, orientation_drone = sim.simxGetObjectOrientation(clientID, drone, -1, sim.simx_opmode_streaming)
res, location_drone = sim.simxGetObjectPosition(clientID, drone, -1, sim.simx_opmode_streaming)
res, location_target = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_streaming)


while 1:
    while start_sim:
        mass = massage.get_massage()
        if mass[0] == 1:
            start_sim = True
            res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
            #time.sleep(3)
            massage.send_massage(comands['sensor_ready'], 1, IP['manager'], ports['manager'])
        elif mass[0] == 17:
            sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
            start_sim = False
        elif mass[0] == comands['stop_sim']:
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        elif mass[0] == comands['send_U']:
            sim.simxSetStringSignal(clientID, 'control_signal', sim.simxPackFloats(mass[1::]), sim.simx_opmode_oneshot)

        res, gps = sim.simxGetStringSignal(clientID, 'gps', sim.simx_opmode_buffer)
        gps = sim.simxUnpackFloats(gps) # 3 floats
        res, gyro = sim.simxGetStringSignal(clientID, 'gyro', sim.simx_opmode_buffer)
        gyro = sim.simxUnpackFloats(gyro) # 3 floats
        res, accel = sim.simxGetStringSignal(clientID, 'nydavai', sim.simx_opmode_buffer)
        accel = sim.simxUnpackFloats(accel)  # 3 floats
        res, orientation_target = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_buffer)
        res, orientation_drone = sim.simxGetObjectOrientation(clientID, drone, -1, sim.simx_opmode_buffer)
        res, location_drone = sim.simxGetObjectPosition(clientID, drone, -1, sim.simx_opmode_buffer)
        res, location_target = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
        res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)
        print(struct.calcsize('>{}h'.format(64*64*3)))
        #struct.pack('>h'.romat(64*64*3))

        # Bool, 3 floats, _ , 3 floats
        res, h_detected, h_point, _, h_norm = sim.simxReadProximitySensor(clientID, h_sensor, sim.simx_opmode_buffer)

        massage.send_massage(comands['send_GPS'], gps, IP['manager'], ports['manager']) #1
        massage.send_massage(comands['send_gyro'], gyro, IP['manager'], ports['manager']) #1
        massage.send_massage(comands['send_accs'], accel, IP['manager'], ports['manager']) #1
        massage.send_massage(comands['send_orientation_drone'], orientation_drone, IP['manager'], ports['manager'])
        massage.send_massage(comands['send_orientation_target'], orientation_target, IP['manager'], ports['manager'])
        massage.send_massage(comands['send_target_pos'], location_target, IP['manager'], ports['manager'])
        massage.send_massage(comands['position drone'], location_drone, IP['manager'], ports['manager'])
        massage.send_massage(comands['send_video'], image, IP['manager'], ports['manager'], resolution)
        massage.send_massage(comands['send_H_sens'], [h_detected, h_point, h_norm], IP['manager'], ports['manager']) #1
        im = array.array('b', image)
        im.reverse()

    mass = massage.get_massage()
    if mass[0] == 23:
        res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        start_sim = True



