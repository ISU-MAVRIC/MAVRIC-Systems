import serial
import time

com_port_name = "COM7"

com_port = serial.Serial(
    port=com_port_name,
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1)

print(com_port.isOpen())
com_port.write("Q\r\n".encode())
time.sleep(2)
print(com_port.read(8))
com_port.write("S\r\n".encode())

def convByte(b):
    if b > 127:
        return (256-b) * (-1)
    else:
        return b

file = com_port #=open("acq3.data", 'rb')
print(file.read(8)) # "S\r\nACK\r\n"
values = [];

while True:
    byte = file.read(1)
    if len(byte)==0:
        break
    else:
        byte = byte[0]
        
    if byte == 0x80:
        result = file.read(2)
        values.append(int(result[0]) << 8 | result[1])
    
    else:
        values.append(values[-1]+convByte(byte))
        pass
file.close()

file=open("acq.csv", 'w')
for value in values:
    # write it to a file as text with a comma after it
    file.write(str(value)+",")
    
file.close()
