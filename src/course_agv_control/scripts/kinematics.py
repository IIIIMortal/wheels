#!/usr/bin/env python

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std

def Distrubute(data):
    global left_spin
    global right_spin
    r=0.08
    l=r/3+0.1/2
    
    print("publish command : vx - %.2f vw - %.2f"%(data.linear.x,data.angular.z))
    xi = data.linear.x
    zi = data.angular.z
    left_spin = (xi-l*zi)/r
    right_spin = (xi+l*zi)/r
    
    #发送
    left_w.publish(left_spin)
    right_w.publish(right_spin)

def transporter():
    global right_w
    global left_w
    rospy.init_node('transporter',anonymous=True)   
    #接受
    data = rospy.Subscriber('/course_agv/velocity',geo.Twist,Distrubute)
    
    right_w = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command',std.Float64,queue_size=1)
    left_w = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command',std.Float64,queue_size=1)
    rospy.spin()



if __name__=='__main__':
    transporter()
