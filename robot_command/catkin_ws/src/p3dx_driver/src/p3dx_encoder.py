#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 

factor_pulse_to_speed = 0.678181*1e-7
inter_wheels_distance = 0.4
#inter wheels de 0.104 pour le kheperas

pub_left = rospy.Publisher('pioneer_p3dx/leftWheelCommand', Float64,queue_size=10)
pub_right = rospy.Publisher('pioneer_p3dx/rightWheelCommand', Float64,queue_size=10)

def callback(msg):    
    linear = msg.linear.x
    angular = msg.angular.z
    
    Lw = inter_wheels_distance * angular
    
    left_wheel_linear_speed = (2.0 * linear - Lw) / 2.0
    right_wheel_linear_speed = (2.0 * linear + Lw) / 2.0
    
    left_wheel_pulses = int(left_wheel_linear_speed / (factor_pulse_to_speed))
    right_wheel_pulses = int(right_wheel_linear_speed / (factor_pulse_to_speed))
    
    print('right wheel pulses {}'.format(right_wheel_pulses))
    print('left wheel pulses {}'.format(left_wheel_pulses))
    pub_left.publish(left_wheel_linear_speed)
    pub_right.publish(right_wheel_linear_speed)


if __name__ == '__main__':
     rospy.init_node('p3dx_wheelsDriver_node')
     rospy.Subscriber("pioneer_p3dx/wheelsDriver", Twist, callback)
     rospy.spin()
