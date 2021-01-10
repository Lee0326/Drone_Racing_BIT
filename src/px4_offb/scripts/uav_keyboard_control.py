#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import sys, select, termios, tty ,string

# msg = """
# Control The Robot!
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
    
    activity = State()

    rospy.init_node('uav_teleop')
    rate = rospy.Rate(20) # 10hz

    uav1SetVelocity_pub =  rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=1)
    activity_pub = rospy.Publisher("uav/activity",State,queue_size=1)


    try:
        while not rospy.is_shutdown():
            key = getKey()
                          
            if key == 'l' :
                
                activity.mode = "land"
                print "1111"
                activity_pub.publish(activity)

            if (key == '\x03'):
                    break

            rate.sleep()

    except:
        print "except"

    finally:
        print "finally"
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
