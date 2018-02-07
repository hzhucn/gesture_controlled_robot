#!/usr/bin/env python
import rospy
import telnetlib
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 

factor_pulse_to_speed = 0.678181*1e-3
inter_wheels_distance = 0.104
ip, port = 'khepera2.smart.metz.supelec.fr', 4100

#inter wheels de 0.104 pour le kheperas


class telnetlib_connection:
    
    def __init__(self,ip,port):
        try : 
            self.tn = telnetlib.Telnet(ip, port)
        except Exception as e:
            print("Impossible de se connecter: {}".format(e))
            sys.exit()

    def write_prediction(self,command):
        try:
            self.tn.write(command.encode('ascii')+ b"\r")
            output = self.tn.read_some()
            print("command received : {}".format(output))
        except Exception as e:
            print("Impossible d'envoyer un message: {}".format(e))
            self.tn.close()

def callback(msg):    
    linear = msg.linear.x
    angular = msg.angular.z
    
    Lw = inter_wheels_distance * angular
    
    left_wheel_linear_speed = (2.0 * linear - Lw) / 2.0
    right_wheel_linear_speed = (2.0 * linear + Lw) / 2.0
    
    left_wheel_pulses = int(left_wheel_linear_speed / (factor_pulse_to_speed))
    right_wheel_pulses = int(right_wheel_linear_speed / (factor_pulse_to_speed))
    command = "D,"+str(left_wheel_pulses)+","+str(right_wheel_pulses)
    print('right wheel pulses {}'.format(right_wheel_pulses))
    print('left wheel pulses {}'.format(left_wheel_pulses))
    
    tn.write_prediction(command)
    
    #pub_left.publish(left_wheel_linear_speed)
    #pub_right.publish(right_wheel_linear_speed)


if __name__ == '__main__':
     tn = telnetlib_connection(ip, port)
     rospy.init_node('kheperaIV_wheelsDriver_node')
     rospy.Subscriber("kheperaIV/wheelsDriver", Twist, callback)
     rospy.spin()
