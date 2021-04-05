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

    def __init__(self, ip):
        self._gps = GPS(ip, 8001)
        self._ip = ip
        self._temperature_getter = ValueGetter(ip, 8002, float)
        self._arm_getter = ValueGetter(ip, 10002, int)
        self._voltage_getter = ValueGetter(ip, 8003, float)
        self._steer_getter = ValueGetter(ip, 9005, int)

    def open(self):
        self._temperature_getter.start()
        self._arm_getter.start()
        self._voltage_getter.start()
        self._steer_getter.start()
        self._gps.start()

    def close(self):
        if self._gps is not None:
            self._gps.close()
        if self._temperature_getter is not None:
            self._temperature_getter.close()
        if self._arm_getter is not None:
            self._arm_getter.close()
        if self._voltage_getter is not None:
            self._voltage_getter.close()
        if self._steer_getter is not None:
            self._steer_getter.close()

    def kill_all(self):
        self.disable_autonomous()

        self.set_wheels(0, 0, 0, 0)
        self.set_arm_base_rot(0)
        self.set_arm_base_pitch(0)
        self.set_arm_elbow_pitch(0)
        self.set_arm_claw_rot(0)
        self.set_arm_claw_pitch(0)
        self.set_arm_claw_actuation(0)

    def set_wheels(self, left, right, strleft, strright):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9002))
            s.sendall(('D' + str(left) + ',' + str(right) + ',' + str(strleft) + ',' + str(strright)).encode())
            s.close()
        except socket.error as e:
            print(e)

    def set_actuator(self, direction):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 10001))
            s.sendall(('RSF'[direction + 1]).encode())
            s.close()
        except socket.error as e:
            print(e)

    def set_arm(self, motor_str, rate):
        try:
            rate_str = str(int(rate))

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 10001))
            s.sendall((motor_str + rate_str).encode())
            s.close()
        except socket.error as e:
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

    def enable_autonomous(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9002))
            s.sendall('NE'.encode())
            s.close()

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9003))
            s.sendall('NE'.encode())
            s.close()

        except socket.error as e:
            print(e)

    def disable_autonomous(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9002))
            s.sendall('ND'.encode())
            s.close()

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9003))
            s.sendall('ND'.encode())
            s.close()
        except socket.error as e:
            print(e)

    def add_waypoint(self, latitude, longitude):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9003))
            s.sendall(('NW' + str(latitude) + ',' + str(longitude)).encode())
            s.close()
        except socket.error as e:
            print(e)

    @property
    def temperature(self):
        if self._voltage_getter.value is None:
            return 0.01
        else:
            return self._temperature_getter.value

    @property
    def steer_feedback(self):
        return self._steer_getter.value

    @property
    def voltage(self):
        if self._voltage_getter.value is None:
            return [0.01, 0.01]
        else:
            return self._voltage_getter.value

    @property
    def arm_feedback(self):
        return self._arm_getter.value

    @property
    def gps(self):
        return self._gps


class ValueGetter(threading.Thread):
    _socket = None
    _run = True
    _ip = None
    _port = None
    _value = None
    _parser = None

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
                values[i] = self._parser(values[i])

    def run(self):
        buffer = ''
        while self._run:
            try:
                if self._socket is None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(1)
                    self._socket.connect((self._ip, self._port))
                buffer = buffer + self._socket.recv(1024).decode()
                while '\r\n' in buffer:
                    (line, buffer) = buffer.split('\r\n', 1)
                    self._value = self.parse_line(line)

            except socket.error:
                self._socket = None

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
        self._satellites = int(data[6])
        return None

    def run(self):
        buffer = ''
        while self._run:
            try:
                if self._socket is None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(1)
                    self._socket.connect((self._ip, self._port))
                buffer = buffer + self._socket.recv(1024).decode()
                while "\n" in buffer:
                    (line, buffer) = buffer.split('\n', 1)
                    self.parse_line(line)
            except socket.error as e:
                self._socket = None
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

    @property
    def satellites(self):
        return self._satellites
