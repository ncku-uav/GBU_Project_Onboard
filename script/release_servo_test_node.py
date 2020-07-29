#!/usr/bin/env python
# license removed for brevity
import message_filters
import rospy, time
from std_msgs.msg import String,Float64,Bool
from geometry_msgs.msg import TwistStamped
import numpy as np
from numpy import cos,sin,tan
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import ActuatorControl, State, GlobalPositionTarget
from wgs_conversions.srv import WgsConversion

class release_test():
    def __init__(self):
        self.node_name = rospy.get_name()
        # Publicaiton
        self.cmd_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)
        self.servo_state = 0 


if __name__ == '__main__':
    
    rospy.init_node('release_servo_test_node', anonymous=False)
    rate = rospy.Rate(1)# 10hz
    n = release_test()
    n.servo_state = 0   
    cmd = ActuatorControl()    

    while not rospy.is_shutdown() :

        if n.servo_state == 0:
            cmd.group_mix = 1
            cmd.controls[4] = 1 
            cmd.controls[5] = 1
            n.cmd_pub.publish(cmd)
            n.servo_state = 1

        else:
            cmd.group_mix = 1
            cmd.controls[4] = -1 
            cmd.controls[5] = -1
            n.cmd_pub.publish(cmd)
            n.servo_state = 0

        print(n.servo_state)
        rate.sleep()
    rospy.spin()

 

        

       
                        
     
                    
                   
    
 
        

        