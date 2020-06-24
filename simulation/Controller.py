import sim
import struct
import socket
import pandas as pd
import numpy as np

comands = {'start system': 1, 'send_path': 2, 'send_GPS': 3,
           'send_accs': 4, 'send_gyro': 5, 'send_video': 6,
           'send_target_pos': 7, 'send_H_sens': 8, 'sensor_ready': 9,
           'path_ready': 10, 'send_orientation_drone': 11, 'send_orientation_target': 12,
           'send_U': 13, 'controll_ready': 14, 'user_path': 15, 'position drone': 16, 'pause': 17,
           'stop_sim': 18, 'Pid_1_set': 19, 'Pid_2_set': 20, 'Pid_3_set': 21,
           'Pid_4_set': 22, 'RestartSim': 23}

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
            massage = list(struct.unpack('>4f', massage[2::]))
            return code, (*massage)

    def send_massage(self, code, data, IP, port, res=None):
        if code == 13:
            format = '>h4f'
            massage = struct.pack(format, code, *data)
            self.local_socket.sendto(massage, (IP, port))
        elif code == 14:
            format = '>h'
            massage = struct.pack(format, code)
            self.local_socket.sendto(massage, (IP, port))
        else:
            return 'Unexpected format'


class PID_controll:
    def __init__(self, k_p, k_i, k_d, h):
        self.k_i = k_i
        self.k_p = k_p
        self.k_d = k_d
        #self.k_v = k_v
        self.h = h
        self.prev_e = 0
        self.integ_sum = 0
        self.X_hist = []
        self.thrust_hist = []
        self.Y_hist = []
        self.rot_hist = []

    def set_params(self, **kwargs):
        self.k_v = kwargs['k_v']
        self.k_p = kwargs['k_p']
        self.k_d = kwargs['k_d']
        self.k_i = kwargs['k_i']

    def get_output(self, E, V =0):
        U = self.k_p * E + self.k_d * (E - self.prev_e)/self.h + self.k_i * (self.h * E + self.integ_sum)
        self.prev_e = E
        return U

class control_positioon():

    def stabilization(self, target_pos, dron_pos, target_orientatgion, dron_orientation, pid_1, pid_2, pid_3, pid_4):
        output = [0, 0, 0, 0]
        # Vertical control
        thrust = pid_1.get_output(target_pos[2] - dron_pos[2])
        #if thrust > 13:
            #thrust = 13
        pid_1.thrust_hist.append(thrust)
        # Horizontal control
        X = pid_2.get_output(target_pos[0] - dron_pos[0])
        Y = pid_3.get_output(target_pos[1] - dron_pos[1])
        pid_3.Y_hist.append(Y)
        pid_2.X_hist.append(X)        # rotation  касательная к траектории - угол рыскания
        rot = pid_4.get_output(target_orientatgion[0] - dron_orientation[0])
        pid_4.rot_hist.append(rot)
        #thrust += self.k_v * (thrust - speed)
        output[0] = thrust * (1 - Y + X + rot)
        output[1] = thrust * (1 - Y - X - rot)
        output[2] = thrust * (1 + Y - X + rot)
        output[3] = thrust * (1 + Y + X - rot)
        data_frame = pd.DataFrame({'pid1':pid_1.thrust_hist,
                                   'pid2': pid2.X_hist,
                                   'pid3': pid_3.Y_hist,
                                   'pid4': pid_4.rot_hist})
        data_frame.to_csv(r'C:\Users\Denis\ML\data\Cours_proj.csv')
        return output

massage = massage(IP['stabilization'], ports['stabilization'])


while not(massage.get_massage()[0] == 1):
    pass
massage.send_massage(comands['controll_ready'], None, IP['manager'], ports['manager'])
start_sim = True
pid1 = PID_controll(1, 1, 1, 1)
pid2 = PID_controll(1, 1, 1, 1)
pid3 = PID_controll(1, 1, 1, 1)
pid4 = PID_controll(1, 1, 1, 1)
controller = control_positioon()
flag1, flag2, flag3, flag4 = [False for x in range(4)]
target_pos, drone_pos, target_orient, drone_orient = [0 for x in range(4)]
while True:
    while start_sim:
        print('pid 1:/nk_p = {}, k_i = {}, k_d = {}'.format(pid1.k_p, pid1.k_i, pid1.k_d))
        mas = massage.get_massage()
        if mas[0] == 17:
            start_sim = False
        elif mas[0] == 18:
            start_sim = False
        if mas[0] == 7:
            target_pos = [x for x in mas[1::]]
            flag1 = True
        elif mas[0] == 11:
            drone_orient = [x for x in mas[1::]]
            flag2 = True
        elif mas[0] == 12:
            target_orient = [x for x in mas[1::]]
            flag3 = True
        elif mas[0] == 16:
            drone_pos = [x for x in mas[1::]]
            flag4 = True
        elif mas[0] == 19:
            pid1.X_hist = [] 
            k_p, k_i, k_d, k_v = mas[1::]
            pid1.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=k_v)
        elif mas[0] == 20:
            pid2.thrust_hist = []
            k_p, k_i, k_d, k_v = mas[1::]
            pid2.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=k_v)
        elif mas[0] == 21:
            pid3.Y_hist = []
            k_p, k_i, k_d = mas[1::]
            pid3.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)
        elif mas[0] == 22:
            pid4.rot_hist = []
            k_p, k_i, k_d = mas[1::]
            pid4.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)

        if flag1 and flag2 and flag3 and flag4:
            output = controller.stabilization(target_pos, drone_pos, target_orient, drone_orient, pid1, pid2, pid3, pid4)
            massage.send_massage(comands['send_U'], output, IP['manager'], ports['manager'])
            print(output)
            flag1, flag2, flag3, flag4 = [False for x in range(4)]
    command = massage.get_massage()
    if command[0] == 23:
        start_sim = True
