#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = "This teleop doesn't do anything yet. I just want the orobot to stand still"


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('flat_drone_teleop')
     
    arm_body_pub = rospy.Publisher('/flat_drone/arm_body_controller/command', Float64, queue_size=10) 
    arm_mid_pub = rospy.Publisher('/flat_drone/arm_mid_controller/command', Float64, queue_size=10) 
    arm_end_pub = rospy.Publisher('/flat_drone/arm_end_controller/command', Float64, queue_size=10) 
    right_finger_pub = rospy.Publisher('/flat_drone/right_finger_controller/command', Float64, queue_size=10) 
    left_finger_pub = rospy.Publisher('/flat_drone/left_finger_controller/command', Float64, queue_size=10)

    arm_body= 0
    arm_mid= 0
    arm_end= 0
    right_finger= 0
    left_finger= 0

    try:
        print(msg)
        while(1):
            arm_body_pub.publish(arm_body) 
            arm_mid_pub.publish(arm_mid)
            arm_end_pub.publish(arm_end)
            right_finger_pub.publish(right_finger)
            left_finger_pub.publish(left_finger)

    except e:
        print(e)

    finally:
        arm_body_pub.publish(arm_body) 
        arm_mid_pub.publish(arm_mid)
        arm_end_pub.publish(arm_end)
        right_finger_pub.publish(right_finger)
        left_finger_pub.publish(left_finger)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)