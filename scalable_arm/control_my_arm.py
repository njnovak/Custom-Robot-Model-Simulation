#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Remote Operations Commencing!
---------------------------
Gripping...
Now!

"""

speed = 800
turn = -0.5

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')

    pub_arm1 = rospy.Publisher('/scalable_arm/arm_1_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    pub_arm2 = rospy.Publisher('/scalable_arm/arm_2_controller/command', Float64, queue_size=10)
    pub_arm3 = rospy.Publisher('/scalable_arm/arm_3_controller/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
    pub_arm4 = rospy.Publisher('/scalable_arm/arm_4_controller/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
    pub_arm5 = rospy.Publisher('/scalable_arm/arm_5_controller/command', Float64, queue_size=10)
    pub_arm6 = rospy.Publisher('/scalable_arm/arm_6_controller/command', Float64, queue_size=10)
    pub_f1 = rospy.Publisher('/scalable_arm/finger_1_controller/command', Float64, queue_size=10)
    pub_f2 = rospy.Publisher('/scalable_arm/finger_2_controller/command', Float64, queue_size=10)

    x = 1
    th = -0.5
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    def vels(speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    print(msg)
    print(vels(speed+x,turn+th))
    
    phi = 0

    

    while(phi<1):
        if(rospy.is_shutdown()):
                print('\nShutdown called.')
                break
        try:
            
            control_turn = th + turn
            control_speed = x + speed

            pub_arm1.publish(0) # publish the turn command.
            pub_arm2.publish(phi) # publish the turn command.
            pub_arm3.publish(0) # publish the control speed. 
            pub_arm4.publish(phi) # publish the control speed. 
            pub_arm5.publish(0) # publish the turn command.
            pub_arm6.publish(phi) # publish the turn command.
            pub_f1.publish(phi) # publish the turn command.
            pub_f2.publish(phi) # publish the turn command.


        except e:
            print(e)

        finally:
            pub_arm1.publish(0) # publish the turn command.
            pub_arm2.publish(phi) # publish the turn command.
            pub_arm3.publish(0) # publish the control speed. 
            pub_arm4.publish(phi) # publish the control speed. 
            pub_arm5.publish(0) # publish the turn command.
            pub_arm6.publish(phi) # publish the turn command.
            pub_f1.publish(phi) # publish the turn command.
            pub_f2.publish(phi) # publish the turn command.
            
        phi+=0.1
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.spin