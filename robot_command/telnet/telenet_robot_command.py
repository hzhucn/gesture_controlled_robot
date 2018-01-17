#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
 
def make_tcp_request(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try :         
        sock.connect((ip, port))
    except Exception as e: 
        print("Impossible de se connecter au serveur: {}".format(e)) 
    try:
        sock.sendall(message.encode('utf-8'))
        response = sock.recv(1024)
        print("Received: {}".format(response))
    except Exception as e:
        print("Impossible d'envoyer un message: {}".format(e))
    finally:
        sock.close()