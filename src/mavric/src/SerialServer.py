import serial
import socket
import select

com_port_name = "COM9"
server_address = ('localhost', 10000)

com_port = serial.Serial(
    port=com_port_name,
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=0)
print(com_port.isOpen())

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.bind(server_address)
socket.listen(1)

connections = []

while True:
 #   print("Checking for new clients\r\n")
    readable, writable, errored = select.select([socket], [], [], 0)
    for socket in readable:
        connection, addr = socket.accept();
        connections.append(connection)
    
    if len(connections) > 0:
        #print("Checking for new network data (%d clients)\r\n" % len(connections))
        readable, writable, errored = select.select(connections, [], connections, 0)
        #print(readable)
        for conn in readable:
            try:
                data = conn.recv(1024).decode()
                if len(data) > 0:
                    com_port.write(data.encode())
            except:
                connections.remove(conn)    
        for conn in errored:
            connections.remove(conn)

#    print("Checking for new serial data\r\n")
    data = com_port.read(1024);
    if len(data) > 0:
        for conn in connections:
            try:
                conn.sendall(data)
            except:
                connections.remove(conn)

com_port.close()
