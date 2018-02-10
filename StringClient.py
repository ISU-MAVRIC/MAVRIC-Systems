import socket
from threading import *
import time
host ="127.0.0.1"
port = 9004
def ts(str):
   s.sendall(r.encode()) 
   data = ''
#   data = s.recv(1024).decode
   print (data)
while True:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host,port))
	r = raw_input('enter: ')
	ts(s)
	s.close()
