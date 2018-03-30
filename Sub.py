#!/usr/bin/env python

import socket


TCP_IP = ''
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print ('Connection address:', addr)
while 1:
	data = conn.recv(BUFFER_SIZE)
	if data:
		print ("received data:", data.decode())
	if data == "A":
		conn.send("apple")
	conn.send("data received")  # echo
	s.listen(1)
	conn, addr = s.accept()
conn.close()
