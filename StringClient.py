import socket
from threading import *
import time
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host ="127.0.0.1"
port = 9002

def ts(str):
	s.sendall(r.encode()) 
	data = ''
#	data = s.recv(1024).decode()
	print(data)
s.connect((host,port))
while 1:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host,port))
	r = raw_input('enter: ')
	ts(r)

s.close()
#time.sleep(5)

