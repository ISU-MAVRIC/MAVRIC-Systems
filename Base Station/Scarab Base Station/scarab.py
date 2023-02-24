import math
import socket
import threading


class Scarab:
    _temperature_getter = None
    _voltage_getter = None
    _steer_getter = None
    _arm_getter = None
    _run = True
    _gps = None
    _ip = None
    _drive_fix = False
    _arm_fix = False
    _auto_fix = False

    def __init__(self, ip, test):
        self._gps = GPS(ip, 8001)
        self._ip = ip
        self._test = test
        self._temperature_getter = ValueGetter(ip, 8002, float)
        self._arm_getter = ValueGetter(ip, 10002, int)
        self._voltage_getter = ValueGetter(ip, 8003, float)
        self._steer_getter = ValueGetter(ip, 9005, int)

    def open(self):
        self._temperature_getter.start()
        #self._arm_getter.start()
        self._voltage_getter.start()
        #self._steer_getter.start()
        self._gps.start()

    def close(self):
        if self._gps is not None:
            self._gps.close()
            self._gps._good_fix = False
        if self._temperature_getter is not None:
            self._temperature_getter.close()
            self._temperature_getter._good_fix = False
        if self._arm_getter is not None:
            self._arm_getter.close()
            self._arm_getter._good_fix = False
        if self._voltage_getter is not None:
            self._voltage_getter.close()
            self._voltage_getter._good_fix = False
        if self._steer_getter is not None:
            self._steer_getter.close()
            self._steer_getter._good_fix = False

    def kill_all(self):
        self.disable_autonomous()

        self.set_drive(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.set_arm_base_rot(0)
        self.set_arm_base_pitch(0)
        self.set_arm_elbow_pitch(0)
        self.set_arm_claw_rot(0)
        self.set_arm_claw_pitch(0)
        self.set_arm_claw_actuation(0)

    def set_wheels(self, data_str):
        try:
            if self._test is False:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.5)
                s.connect((self._ip, 9002))
                s.sendall(data_str.encode())
                s.close()
                self._drive_fix = True

        except socket.error as e:
            self._drive_fix = False
            print("set wheels")
            print(e)

    def set_drive(self, lf, lm, lb, rf, rm, rb, slf, slb, srf, srb):
        self.set_wheels("D" + str(lf) + ',' + str(lm) + ',' + str(lb) + ',' + str(rf) + ',' + str(rm) + ',' + str(rb)
                        + ',' + str(slf) + ',' + str(slb) + ',' + str(srf) + ',' + str(srb))

    def set_cal(self, clf, clb, crf, crb):
        self.set_wheels("C" + str(clf) + ',' + str(clb) + ',' + str(crf) + ',' + str(crb))

    def set_actuator(self, direction):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect((self._ip, 10001))
            s.sendall(('RSF'[direction + 1]).encode())
            s.close()
        except socket.error as e:
            print("set actuator")
            print(e)

    def set_arm(self, motor_str, rate):
        try:
            if self._test is False:
                rate_str = str(int(rate))
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.25)
                s.connect((self._ip, 10001))
                s.sendall((motor_str + rate_str).encode())
                s.close()
                self._arm_fix = True
        except socket.error as e:
            self._arm_fix = False
            print("set arm")
            print(e)

    def set_arm_base_rot(self, rate):
        self.set_arm('AR', rate)

    def set_arm_base_pitch(self, rate):
        self.set_arm('AL', rate)

    def set_arm_elbow_pitch(self, rate):
        self.set_arm('AE', rate)

    def set_arm_claw_rot(self, rate):
        self.set_arm('CR', rate)

    def set_arm_claw_pitch(self, rate):
        self.set_arm('CP', rate)

    def set_arm_claw_actuation(self, rate):
        self.set_arm('CC', rate)

    def set_arm_all(self, sr, sp, ep, wr, wp):
        try:
            if self._test is False:
                value_str = "S"+str(sr)+","+str(sp)+","+str(ep)+","+str(wr)+","+str(wp)
                #print(value_str)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.25)
                s.connect((self._ip, 10001))
                s.sendall(value_str.encode())
                s.close()
                self._arm_fix = True
        except socket.error as e:
            self._arm_fix = False
            print("set arm all")
            print(e)

    def set_arm_cam(self, hor, vert):
        try:
            if self._test is False:
                value_str = "EV"+str(hor)+","+str(vert)
                #print(value_str)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1)
                s.connect((self._ip, 10001))
                s.sendall(value_str.encode())
                s.close()
                self._arm_fix = True
        except socket.error as e:
            self._arm_fix = False
            print("set arm cam")
            print(e)

    def set_util(self, rate):
        self.set_arm('EU', rate)

    def set_hook(self, rate):
        self.set_arm('EH', rate)

    def set_sci(self, rate):
        self.set_arm('EL', rate)

    def set_arm_enable(self):
        self.set_arm('NE', 1)

    def set_arm_disable(self):
        self.set_arm('ND', 0)

    def set_button(self, rate):
        self.set_arm('EB', rate)

    def set_arm_pos(self, x, y, z, alpha, beta, cl_pos):
        try:
            if self._test is False:
                value_str = "P"+str(x)+","+str(y)+","+str(z)+","+str(alpha)+","+str(beta)+","+str(cl_pos)
                #print(value_str)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1)
                s.connect((self._ip, 10001))
                s.sendall(value_str.encode())
                s.close()
                self._arm_fix = True
        except socket.error as e:
            self._arm_fix = False
            print("set arm pos")
            print(e)

    def enable_autonomous(self):
        try:
            if self._test is False:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect((self._ip, 9002))
                s.sendall('NE'.encode())
                s.close()

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1)
                s.connect((self._ip, 9003))
                s.sendall('NE'.encode())
                s.close()
                self._auto_fix = True

        except socket.error as e:
            self._auto_fix = False
            print("enable autonomous")
            print(e)

    def disable_autonomous(self):
        try:
            if self._test is False:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect((self._ip, 9002))
                s.sendall('ND'.encode())
                s.close()

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1)
                s.connect((self._ip, 9003))
                s.sendall('ND'.encode())
                s.close()
                self._auto_fix = True

        except socket.error as e:
            self._auto_fix = False
            print("disable autonomous")
            print(e)

    def add_waypoint(self, latitude, longitude, post_id):
        try:
            if self._test is False:
                print("Aaaaaaaaaaaaaaaaaaaa")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect((self._ip, 9003))
                s.sendall(('NW' + str(latitude) + ',' + str(longitude) + ',' + str(post_id)).encode())
                s.close()
                self._auto_fix = True

        except socket.error as e:
            self._auto_fix = False
            print("Aaaaaaaaaaaaaaaaaaaa")
            print("add waypoint")
            print(e)

    def forget_waypoints(self):
        try:
            if self._test is False:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect((self._ip, 9003))
                s.sendall('NF'.encode())
                s.close()
                self._auto_fix = True

        except socket.error as e:
            self._auto_fix = False
            print("add waypoint")
            print(e)

    @property
    def temperature(self):
        if self._temperature_getter.value is None:
            return None
        else:
            return self._temperature_getter.value

    @property
    def steer_feedback(self):
        if self._steer_getter.value is None:
            return [None, None, None, None]
        else:
            return self._steer_getter.value

    @property
    def voltage(self):
        if self._voltage_getter.value is None:
            return [None, None]
        else:
            return self._voltage_getter.value

    @property
    def arm_feedback(self):
        return self._arm_getter.value


class ValueGetter(threading.Thread):
    _socket = None
    _run = True
    _ip = None
    _port = None
    _value = None
    _parser = None
    _good_fix = False

    def __init__(self, ip, port, parser):
        self._ip = ip
        self._port = port
        self._parser = parser
        threading.Thread.__init__(self)

    def parse_line(self, line):
        if line.find(',') == -1:
            return line
        else:
            values = line.split(',')
            for i in range(len(values)):
                values[i] = float(self._parser(values[i]))
        return values

    def run(self):
        buffer = ''
        while not self._run:
            try:
                if self._socket is None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(0.5)
                    self._socket.connect((self._ip, self._port))
                    self._good_fix = True
                buffer = buffer + self._socket.recv(1024).decode()
                while '\r\n' in buffer:
                    (line, buffer) = buffer.split('\r\n', 1)
                    self._value = self.parse_line(line)

            except socket.error as e:
                self._socket = None
                self._good_fix = False
                print("value getter "+str(self._port))
                print(e)

    def close(self):
        self._run = False
        self.join()
        if self._socket is not None:
            self._socket.close()

    @property
    def value(self):
        return self._value


class GPS(ValueGetter):
    _good_fix = False
    _latitude = 0.
    _longitude = 0.
    _altitude = 0.
    _speed = 0.
    _heading = 0.
    _satellites = 0.

    def __init__(self, ip, port):
        ValueGetter.__init__(self, ip, port, self.parse_line)

    def parse_line(self, line):
        data = line.split(',')
        self._good_fix = bool(data[0])
        self._latitude = float(data[1])
        self._longitude = float(data[2])
        self._altitude = float(data[3])
        self._speed = float(data[4])
        self._heading = float(data[5])

    def run(self):
        buffer = ''
        while not self._run:
            try:
                if self._socket is None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(0.5)
                    self._socket.connect((self._ip, self._port))
                buffer = buffer + self._socket.recv(1024).decode()
                while "\n" in buffer:
                    (line, buffer) = buffer.split('\n', 1)
                    self.parse_line(line)
            except socket.error as e:
                self._socket = None
                print("GPS data")
                print(e)

    def close(self):
        self._run = False
        self.join()
        if self._socket is not None:
            self._socket.close()

    @property
    def good_fix(self):
        return self._good_fix

    @property
    def latitude(self):
        return self._latitude

    @property
    def longitude(self):
        return self._longitude

    @property
    def altitude(self):
        return self._altitude

    @property
    def speed(self):
        return self._speed

    @property
    def heading(self):
        return self._heading

