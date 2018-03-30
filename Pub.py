#!/usr/bin/env python

import socket


TCP_IP = '10.36.178.114' #IPv4 for receiving computer
TCP_PORT = 5005
BUFFER_SIZE = 1024

def sendMsg(str):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.sendall(str.encode())
	data = s.recv(BUFFER_SIZE)
	print(data.decode());
	s.close()
	return

while 1:
	MESSAGE = input("Enter Command: ")
	sendMsg(MESSAGE)


