import struct
import socket
import time
import sim
import array
import sys
import cv2
from PIL import ImageTk
import PIL.Image as img
import threading
import numpy as np

commands = {'start system': 1, 'send_path': 2, 'send_GPS': 3,
            'send_accs': 4, 'send_gyro': 5, 'send_video': 6,
            'send_target_pos': 7, 'send_H_sens': 8, 'sensor_ready': 9,
            'path_ready': 10, 'send_orientation_drone': 11, 'send_orientation_target': 12,
            'send_U': 13, 'controll_ready': 14, 'user_path': 15, 'position drone': 16, 'pause': 17,
            'stop_sim': 18, 'Pid_1_set': 19, 'Pid_2_set': 20, 'Pid_3_set': 21,
            'Pid_4_set': 22, 'continue': 23, 'start_video': 24, 'Pid_5_set': 25,
            'Pid_6_set': 26, 'velocity': 27}

IP = {'manager': '127.0.0.1', 'obstacle avoidance': '127.0.0.1',
      'Path generator': '127.0.0.1', 'navigation system': '127.0.0.1',
      'object detection': '127.0.0.1', 'inteface': '127.0.0.1',
      'stabilization': '127.0.0.1', 'sensor module': '127.0.0.1'}

ports = {'manager': 703, 'obstacle avoidance': 708,
         'Path generator': 732, 'navigation system': 733,
         'object detection': 734, 'inteface': 735,
         'stabilization': 736, 'sensor module': 746}


def change_values(massage, pid1, pid2, pid3, pid4, pid5, pid6, clientID):
    while 1:
        mass = massage.get_massage()
        if mass[0] == 1:
            res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
            massage.send_massage(commands['sensor_ready'], 1, IP['manager'], ports['manager'])
        elif mass[0] == 17:
            sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
            start_sim = False
        elif mass[0] == commands['stop_sim']:
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        elif mass[0] == 19:
            print('seted')
            k_p, k_i, k_d, k_v = mass[1::]
            pid1.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=k_v)
        elif mass[0] == 20:
            k_p, k_i, k_d = mass[1::]
            pid2.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)
        elif mass[0] == 21:

            k_p, k_i, k_d = mass[1::]
            pid3.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)
        elif mass[0] == 22:

            k_p, k_i, k_d = mass[1::]
            pid4.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)
        elif mass[0] == 25:

            k_p, k_i, k_d = mass[1::]
            pid5.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)
        elif mass[0] == 26:

            k_p, k_i, k_d = mass[1::]
            pid6.set_params(k_p=k_p, k_i=k_i, k_d=k_d, k_v=0)

class PID_controll:
    def __init__(self, k_p, k_i, k_d):
        self.k_i = k_i
        self.k_p = k_p
        self.k_d = k_d
        self.prev_e = 0
        self.integ_sum = 0
        self.X_hist = []
        self.k_v = 0
        self.thrust_hist = []
        self.Y_hist = []
        self.rot_hist = []

    def set_params(self, **kwargs):
        self.k_p = kwargs['k_p']
        self.k_d = kwargs['k_d']
        self.k_i = kwargs['k_i']
        self.k_v = kwargs['k_v']
        self.integ_sum = 0


    def get_output(self, E, h):
        U = self.k_p * E + self.k_d * (E - self.prev_e)/h + self.k_i * (h * E + self.integ_sum)
        self.prev_e = E
        return U


class control_positioon():
    def __init__(self):
        self.size = 50
        self.x_vec = np.linspace(0, 1, self.size+1)[0:-1]
        self.y_vec1 = np.array([1 for _ in range(len(self.x_vec))])
        self.y_vec2 = np.array([1 for _ in range(len(self.x_vec))])
        self.y_vec3 = np.array([1 for _ in range(len(self.x_vec))])
        self.y_vec4 = np.array([1 for _ in range(len(self.x_vec))])
        self.line1 = []
        self.line2 = []
        self.line3 = []
        self.line4 = []

    def rotMatrix(self, alpha, beta, gamma, x, y, z):
        R_x = np.array([[1, 0, 0, x],
                        [0, np.cos(alpha), -np.sin(alpha), 0],
                        [0, np.sin(alpha), np.cos(alpha), 0],
                        [0, 0, 0, 1]])
        R_y = np.array([[np.cos(beta), 0, np.sin(beta), 0],
                        [0, 1, 0, y],
                        [-np.sin(beta), 0, np.cos(beta), 0],
                        [0, 0, 0, 1]])
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],
                        [np.sin(gamma), np.cos(gamma), 0, 0],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])

        M = R_x @ R_y @ R_z
        return M

    def stabilization(self, target_pos, dron_pos, target_orientatgion, dron_orientation, pid_1, pid_2, pid_3,
                      pid_4, pid_5, pid_6, vilocity, h=1):
        output = [0, 0, 0, 0]
        # Vertical control
        thrust = 5.353 + pid_1.get_output(target_pos[2] - dron_pos[2], h) + pid_1.k_v*vilocity

        # Horizontal control
        vx = np.array([1, 0, 0, 1])
        vy = np.array([0, 1, 0, 1])
        M = self.rotMatrix(dron_orientation[0], dron_orientation[1], dron_orientation[2],
                           dron_pos[0], dron_pos[1], dron_pos[2])
        vx = M @ np.transpose(vx)
        vy = M @ np.transpose(vy)
        sim.simxSetStringSignal(clientID, 'Y_signal', sim.simxPackFloats(vy),
                                sim.simx_opmode_oneshot)
        sim.simxSetStringSignal(clientID, 'X_signal', sim.simxPackFloats(vx),
                                sim.simx_opmode_oneshot)

        local_target_pos = np.linalg.inv(M) @ np.array(target_pos + [1])
        sim.simxSetStringSignal(clientID, 'target_signal', sim.simxPackFloats(local_target_pos[:3]), sim.simx_opmode_oneshot)
        alphaE = vy[2] - M[2, 3]
        alpha_cor = pid_5.get_output(alphaE, h)
        betaE = vx[2] - M[2, 3]
        beta_cor = pid_6.get_output(betaE, h)
        alpha_cor = alpha_cor + pid_2.get_output(local_target_pos[1], h)

        beta_cor = beta_cor + pid_3.get_output(local_target_pos[0], h)

        # rotation  касательная к траектории - угол рыскания
        rot = pid_4.get_output(target_orientatgion[2], h)

        output[0] = thrust*(1 - alpha_cor + beta_cor + rot)
        output[1] = thrust*(1 - alpha_cor - beta_cor - rot)
        output[2] = thrust*(1 + alpha_cor - beta_cor + rot)
        output[3] = thrust*(1 + alpha_cor + beta_cor - rot)
        return output


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
        getting_mass = self.local_socket.recv(512)
        code = int.from_bytes(getting_mass[0:2], "big")
        if code == 2 or code == 3 or code == 4 or code == 5 or code == 7 or code == 16 \
                or code == 11 or code == 12 or code == 22\
                or code == 25 or code == 26 or code == 20 or code == 21:
            getting_mass = list(struct.unpack('>3f', getting_mass[2::]))
            return [code, (*getting_mass)]
        elif code == 8:
            getting_mass = list(struct.unpack('>?6f', getting_mass[2::]))
            return [code, (*getting_mass)]
        elif code == 1 or code == 17 or code == 18 or code == 23 or code == 9 or code == 10 or code == 14 or code == 24:
            return [code]
        elif code == 13 or code == 19:
            getting_mass = list(struct.unpack('>4f',  getting_mass[2::]))
            return [code, (*getting_mass)]
        elif code == 15:
            size = int.from_bytes(getting_mass[2:4], "big")
            print(f'size = {size}')
            massage = list(struct.unpack(f'>{size}h', getting_mass[4::]))
            return [code, (*massage)]


    def send_massage(self, code, data, IP, port, res = None):
        if data:
            if code == 2 or code == 12 or code == 4 or code == 5 or code == 7 or code == 16 or code == 11 \
                    or code == 3 or code == 27:
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
                format = '>hhh' + str(res[0]*res[1]*3) + 'h' # code res, data
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

_, drone = sim.simxGetObjectHandle(clientID, 'my_drone', sim.simx_opmode_blocking)
_, target = sim.simxGetObjectHandle(clientID, 'Quadcopter_target', sim.simx_opmode_blocking)
_, camera = sim.simxGetObjectHandle(clientID, 'Main_camera', sim.simx_opmode_oneshot_wait)
_, h_sensor = sim.simxGetObjectHandle(clientID, 'H_sensor', sim.simx_opmode_oneshot_wait)
_, gps = sim.simxGetStringSignal(clientID, 'gps', sim.simx_opmode_streaming)
_, gyro = sim.simxGetStringSignal(clientID, 'gyro', sim.simx_opmode_streaming)
_, accel = sim.simxGetStringSignal(clientID, 'nydavai', sim.simx_opmode_streaming)
_, velocity, _ = sim.simxGetObjectVelocity(clientID, drone, sim.simx_opmode_streaming)

_, euler = sim.simxGetObjectOrientation(clientID, drone, target, sim.simx_opmode_streaming)
_, h_detected, h_point, h_det_obj, h_norm = sim.simxReadProximitySensor(clientID, h_sensor, sim.simx_opmode_streaming)
_, orientation_target = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_streaming)
_, orientation_drone = sim.simxGetObjectOrientation(clientID, drone, -1, sim.simx_opmode_streaming)
_, location_drone = sim.simxGetObjectPosition(clientID, drone, -1, sim.simx_opmode_streaming)
_, location_target = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
_, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_streaming)

pid1 = PID_controll(2, 0, 0)
pid1.k_v = -2
pid2 = PID_controll(0.005, 0, 1)
pid3 = PID_controll(-0.005, 0, -1)
pid4 = PID_controll(0.1, 0, 2)
pid5 = PID_controll(0.25, 0, 2.1)
pid6 = PID_controll(-0.25, 0, -2.1)
controller = control_positioon()
pev_time = 0
flag1, flag2, flag3, flag4, flag5 = [False for _ in range(5)]
target_pos, drone_pos, target_orient, drone_orient = [0 for _ in range(4)]
lock = threading.Lock()
t1 = threading.Thread(target=change_values, args=[massage, pid1, pid2, pid3, pid4, pid5, pid6, clientID])
t1.start()
while 1:
    while start_sim:
        _, gps = sim.simxGetStringSignal(clientID, 'gps', sim.simx_opmode_buffer)
        gps = sim.simxUnpackFloats(gps) # 3 floats
        _, gyro = sim.simxGetStringSignal(clientID, 'gyro', sim.simx_opmode_buffer)
        gyro = sim.simxUnpackFloats(gyro) # 3 floats
        _, accel = sim.simxGetStringSignal(clientID, 'nydavai', sim.simx_opmode_buffer)
        accel = sim.simxUnpackFloats(accel)  # 3 floats
        _, orientation_target = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_buffer)
        _, orientation_drone = sim.simxGetObjectOrientation(clientID, drone, -1, sim.simx_opmode_buffer)
        _, location_drone = sim.simxGetObjectPosition(clientID, drone, -1, sim.simx_opmode_buffer)
        _, location_target = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
        _, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)
        _, velocity, _ = sim.simxGetObjectVelocity(clientID, drone, sim.simx_opmode_buffer)
        _, euler = sim.simxGetObjectOrientation(clientID, drone, target, sim.simx_opmode_buffer)


        # Bool, 3 floats, _ , 3 floats
        _, h_detected, h_point, _, h_norm = sim.simxReadProximitySensor(clientID, h_sensor, sim.simx_opmode_buffer)
        massage.send_massage(commands['velocity'], velocity, IP['manager'], ports['manager'])
        massage.send_massage(commands['send_GPS'], gps, IP['manager'], ports['manager']) #1
        massage.send_massage(commands['send_gyro'], gyro, IP['manager'], ports['manager']) #1
        massage.send_massage(commands['send_accs'], accel, IP['manager'], ports['manager']) #1

        massage.send_massage(commands['position drone'], location_drone, IP['manager'], ports['manager'])
        massage.send_massage(commands['send_H_sens'], [h_detected, h_point, h_norm], IP['manager'], ports['manager'])
        if velocity:
            lock.acquire()
            try:
                h = 1 #(time.time_ns() - cur_time + 1e-7) / 1e9
                if location_target and location_drone and orientation_target and orientation_drone:
                    output = controller.stabilization(location_target, location_drone, euler, orientation_drone,
                                                  pid1, pid2, pid3,
                                                  pid4, pid5, pid6, velocity[2], h)
                    sim.simxSetStringSignal(clientID, 'control_signal', sim.simxPackFloats(output), sim.simx_opmode_oneshot)
                cur_time = time.time_ns()
            finally:
                lock.release()

        cur_time = time.time_ns() / (10 ** 6)
        print(cur_time - pev_time)
        pev_time = cur_time
        time.sleep(20/1e3)
    mass = massage.get_massage()

    if mass[0] == 23:
        res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        start_sim = True



'''elif mass[0] == commands['start_video']:
                show_cam = True
                if res == sim.simx_return_ok:
                    im = array.array('b', image)
                    im.reverse()
                    im = img.frombuffer('RGB', resolution, bytes(im), 'raw', 'RGB', 0, 1)
                    image_cv = np.asarray(im)
                    cv2.flip(image_cv, 1, image_cv)
                    cv2.imshow('im', image_cv)
                    #k = cv2.waitKey(1)'''
