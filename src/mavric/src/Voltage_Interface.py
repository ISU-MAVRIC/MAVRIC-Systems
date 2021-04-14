# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 15:29:55 2021

@author: Keith Harder
"""

import socket
import rospy
from std_msgs.msg import String
from mavric.msg import Voltage

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 8003
print (host)
print (port)

clients = [];

def callback(data):
   
    
    message = '%d,%d\r\n' % (data.batt1,data.batt2)
    
    for client in clients:
        try:
            client.sendall(message.encode())
            
        except socket.error as e:
            print(e)
            client.close();
            clients.remove(client)
            print('client removed')
            print('%d clients remain\n' % len(clients))
    
def listener():
    
    rospy.init_node('listener', anonymous = True)
    port = rospy.get_param("~port", 8003)
    rospy.Subscriber("Voltage_Monitor", Voltage, callback, queue_size = 10)
    
    serversocket.bind((host, port))
    serversocket.listen(1)
    rospy.loginfo('Voltage socket connected')
    while not rospy.is_shutdown():
        connection, adress = serversocket.accept()
        clients.append(connection)
        print('we have a voltge monitor listener: ')
        print(adress)
        print('\n')


    for client in clients:
        client.close()
    serversocket.close()
    
    

if __name__ == '__main__':
    listener()
    
'''steer_feedback'''
