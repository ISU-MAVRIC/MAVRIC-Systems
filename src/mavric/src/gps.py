import serial
import time

class GPS:
    _data = []
    _port = serial.Serial()

    #configure serial port    
    def __init__(self, port, baud=9600, timeout=1):
        self._port.port = port
        self._port.baudrate = 9600
        self._port.bytesize = serial.EIGHTBITS
        self._port.parity = serial.PARITY_NONE
        self._port.stopbits = serial.STOPBITS_ONE
        self._port.xonxoff = True
        self._port.timeout = timeout

    #open serial port and turn GPS module on
    def begin(self):
        self._port.open()
        self._port.write(b'AT+CGNSPWR=1\r\n')
        time.sleep(0.1)

    #read GPS data
    def update(self):
        self._port.write(b'AT+CGNSINF\r\n')
        time.sleep(0.1)

        while self._port.in_waiting > 0:
            in_str = self._port.readline()
            print(in_str)
            if '+CGNSINF:' in in_str:
                self._data = in_str.replace('+CGNSINF: ', '').split(',')
                return

    #turn GPS module off and close serial port
    def close(self):
        self._port.write(b'AT+CGNSPWR=0\r\n')
        time.sleep(0.1)
        
        self._port.close()

    @property
    def good_fix(self):
        if (self._data == [] or self._data[1] == ""):
            return False
        return bool(int(self._data[1])) #returns 1 if gps position is valid

    @property
    def date(self):
	if(self._data == [] or self._data[2] == ""):
		return [0,0,0]

        y = int(self._data[2][0:4])
        m = int(self._data[2][4:6])
        d = int(self._data[2][6:8])

        return [y,m,d]                  #returns date in the form [year, month, day]

    @property
    def time(self):
	if(self._data == [] or self._data[2] == ""):
		return [0,0,0]

        h = int(self._data[2][8:10])
        m = int(self._data[2][10:12])
        s = float(self._data[2][12:18])

        return [h,m,s]                  #returns GMT time in the form [hour, min, sec]
    
    @property
    def latitude(self):
        if(self._data == [] or self._data[3] == ""):
            return 0
        
        return float(self._data[3])     #returns latitude in decimal-degrees

    @property
    def longitude(self):
        if(self._data == [] or self._data[4] == ""):
            return 0
        
        return float(self._data[4])     #returns longitude in decimal-degrees

    @property
    def altitude(self):
        if (self._data == [] or self._data[5] == ""):
            return 0
        
        return float(self._data[5])     #returns altitude in meters

    @property
    def speed(self):
        if (self._data == [] or self._data[6] == ""):
            return 0
        
        return float(self._data[6])     #returns ground speed in km / h

    @property
    def heading(self):
        if (self._data == [] or self._data[7] == ""):
            return 0
        
        return float(self._data[7])     #returns heading in degrees clockwise from true north

    @property
    def satellites(self):
        if (self._data == [] or self._data[15] == ""):
            return 0
        
        return int(self._data[14])      #returns number of visible satellites
