import serial
import socket
import select

com_port_name = "COM10"
server_address = ('localhost', 10000)

com_port = serial.Serial(
    port=com_port_name,
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=0)
print(com_port.isOpen())

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)

while True:
    readable, writable, errored = select.select([sock], [], [sock], 0)
    for conn in readable:
        data = conn.recv(1024).decode()
        com_port.write(data.encode())
    for conn in errored:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(server_address)

    data = com_port.read(1024);
    if len(data) > 0:
        try:
            sock.sendall(data)
        except socket.error as e:
            print(e)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(server_address)

com_port.close()
