#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = "This teleop doesn't do anything yet. I just want the orobot to stand still"
# """
# Control Your Toy!
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .
# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%
# space key, k : force stop
# anything else : stop smoothly
# CTRL-C to quit
# """

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('quad_bot_teleop')
     
    joint_motor_pub = rospy.Publisher('/build_quad_bot/joint_motor_controller/command', Float64, queue_size=10) 
    arm_body_pub = rospy.Publisher('/build_quad_bot/arm_body_controller/command', Float64, queue_size=10) 
    arm_mid_pub = rospy.Publisher('/build_quad_bot/arm_mid_controller/command', Float64, queue_size=10) 
    arm_end_pub = rospy.Publisher('/build_quad_bot/arm_end_controller/command', Float64, queue_size=10) 
    right_finger_pub = rospy.Publisher('/build_quad_bot/right_finger_controller/command', Float64, queue_size=10) 
    left_finger_pub = rospy.Publisher('/build_quad_bot/left_finger_controller/command', Float64, queue_size=10)

    thrust = 0
    arm_body_theta = 0
    arm_mid_theta = 0
    arm_end_theta = 0
    right_finger_theta = 0
    left_finger_theta = 0
    try:
        print(msg)
        while(1):
            joint_motor_pub.publish(thrust)
            arm_body_pub.publish(arm_body_theta) 
            arm_mid_pub.publish(arm_mid_theta)
            arm_end_pub.publish(arm_end_theta)
            right_finger_pub.publish(right_finger_theta)
            left_finger_pub.publish(left_finger_theta)


    except e:
        print(e)

    finally:
        joint_motor_pub.publish(thrust)
        arm_body_pub.publish(arm_body_theta)
        arm_mid_pub.publish(arm_mid_theta)
        arm_end_pub.publish(arm_end_theta)
        right_finger_pub.publish(right_finger_theta)
        left_finger_pub.publish(left_finger_theta)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)