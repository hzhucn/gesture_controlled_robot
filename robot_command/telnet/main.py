#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import telnetlib


ip, port = 'khepera2.smart.metz.supelec.fr', 4100
command = "D,0,0/r"
try: 
    tn = telnetlib.Telnet(ip, port)
    tn.write(command.encode('utf-8'))
    output = tn.read_some()
    print(output)
except Exception as e:
        print("Impossible d'envoyer un message: {}".format(e))
tn.close()

