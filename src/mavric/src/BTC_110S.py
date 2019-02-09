import serial

def convByte(b):
        if b > 127:
            return (256-b) * (-1)
        else:
            return b
        
class Spectrometerdriver:
    
    _port = serial.Serial()
    
    def __init__(self, port):
        self._port.port = port
        self._port = serial.Serial(
            port     = port,
            baudrate = 9600,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout  = 1)
        
    
    def reset(self):
        self._port.write("Q\r\n".encode())
        response = self._port.read(8).decode()
        if response != "Q\r\nACK\r\n":
            return False;
        return True;
    
    def scan(self):
        self._port.write("S\r\n".encode())
        response = self._port.read(8).decode()
        if response != "S\r\nACK\r\n":
            return False;
        
        values = [];

        while True:
            byte = self._port.read(1)
            if len(byte)==0:
                break
            else:
                byte = byte[0]
                
            if byte == 0x80:
                result = self._port.read(2)
                values.append(int(result[0]) << 8 | result[1])
            
            else:
                values.append(values[-1]+convByte(byte))
        return values;
    
    def close(self):
        self._port.close()
        

