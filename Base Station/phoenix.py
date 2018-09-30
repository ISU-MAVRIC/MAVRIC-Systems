import math
import socket
import threading

class Phoenix:
    _temperature_getter = None
    _actuator_getter = None
    _run = True
    _gps = None
    _ip = None
    
    def __init__(self, ip):
        self._gps = GPS(ip, 8001);
        self._ip = ip
        self._temperature_getter = ValueGetter(ip, 8002, float)
        self._actuator_getter = ValueGetter(ip, 10002, float)
    
    def open(self):
        self._temperature_getter.start()
        self._actuator_getter.start()
        self._gps.start()
    
    def close(self):
        if self._gps != None:
            self._gps.close()
        if self._temperature_getter != None:
            self._temperature_getter.close()
        if self._actuator_getter != None:
            self._actuator_getter.close()
        
    def setWheels(self, left, right):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 9002))
            s.sendall(('D'+str(left)+','+str(right)).encode())
            s.close()
        except socket.error as e:
            print(e)
            
    def setActuator(self, direction):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((self._ip, 10001))
            s.sendall(('RSF'[direction+1]).encode())
            s.close()
        except socket.error as e:
            print(e)

    @property
    def temperature(self):
        return self._temperature_getter.value
    
    @property
    def actuator(self):
        return self._actuator_getter.value
    
    @property
    def gps(self):
        return self._gps

class ValueGetter (threading.Thread):
    _socket = None
    _run = True
    _ip = None
    _port = None
    _value = None
    _parser = None
    
    def __init__(self, ip, port, parser):
        self._ip = ip
        self._port = port
        threading.Thread.__init__(self)
        self._parser = parser
        
    def run(self):
        buffer = ''
        while self._run:
            try:
                if self._socket == None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(1)
                    self._socket.connect((self._ip, self._port))
                buffer = buffer + self._socket.recv(1024).decode()
                while '\r\n' in buffer:
                    (line, buffer) = buffer.split('\r\n', 1)
                    self._value = self._parser(line)
                                    
            except socket.error:
                self._socket = None
            
    def close(self):
        self._run = False
        self.join()
        if self._socket != None:
            self._socket.close()
        
    @property
    def value(self):
        return self._value
        
class GPS (ValueGetter):
    _good_fix = False
    _latitude = 0.
    _longitude = 0.
    _altitude = 0.
    _speed = 0.
    _heading = 0.
    _satelites = 0.
    
    def __init__(self, ip, port):
        ValueGetter.__init__(self, ip, port, self.parseLine)
        
    def parseLine(self, line):
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
                if self._socket == None:
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._socket.settimeout(1)
                    self._socket.connect((self._ip, self._port))
                buffer = buffer + self._socket.recv(1024).decode()
                while "\n" in buffer:
                    (line, buffer) = buffer.split('\n', 1)
                    self.parseLine(line)
            except socket.error as e:
                self._socket = None

    def close(self):
        self._run = False
        self.join()
        if self._socket != None:
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
