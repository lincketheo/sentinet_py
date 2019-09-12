#!/usr/bin/env python
import numpy as np
import rospy
import geometry_msgs.msg

global active_tags
global Position
active_tags=[0,0,0,0]
Position=[[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan]]

def callback0(data):
    global Position
    global active_tags
    if data.y != -42.0:
        active_tags[0]=1
        Position[0][0]=data.x
        Position[0][1]=data.y
        Position[0][2]=data.z

def callback1(data):
    global Position
    global active_tags
    if data.y != -42:
        active_tags[1]=1
        Position[1][0]=data.x
        Position[1][1]=data.y
        Position[1][2]=data.z

def callback2(data):
    global Position
    global active_tags
    if data.y != -42:
        active_tags[2]=1
        Position[2][0]=data.x
        Position[2][1]=data.y
        Position[2][2]=data.z

def callback3(data):
    global Position
    global active_tags
    if data.y != -42:
        active_tags[3]=1
        Position[3][0]=data.x
        Position[3][1]=data.y
        Position[3][2]=data.z

def pos_sub():
    rospy.Subscriber('Bot_Pos0',geometry_msgs.msg.Vector3,callback0)
    rospy.Subscriber('Bot_Pos1',geometry_msgs.msg.Vector3,callback1)
    rospy.Subscriber('Bot_Pos2',geometry_msgs.msg.Vector3,callback2)
    rospy.Subscriber('Bot_Pos3',geometry_msgs.msg.Vector3,callback3)

def pubsub():
    global Position
    global active_tags
    pubber=rospy.Publisher('Bot_Pos',geometry_msgs.msg.Twist, queue_size=1)
    rospy.init_node('pubber',anonymous=True)
    rate=rospy.Rate(10)
    # Y POSITION IS ANGLE OFF X AXIS IN RADIANS. ANGULAR IS ERROR IN RESPECTIVE TERMS
    # if no tags can be seen position is -42,-42,-42 w/ error of 0
    while not rospy.is_shutdown():
        pos_est=geometry_msgs.msg.Twist()
        xac=[np.nan,np.nan,np.nan,np.nan]
        yac=[np.nan,np.nan,np.nan,np.nan]
        zac=[np.nan,np.nan,np.nan,np.nan]
        act=sum(active_tags)
        if act>0:
            for i in range(0,3):
                xac[i]=Position[i][0]
                yac[i]=Position[i][1]
                zac[i]=Position[i][2]
            pos_est.linear.x=np.nanmean(xac)
            pos_est.linear.y=np.nanmean(yac)
            pos_est.linear.z=np.nanmean(zac)
            pos_est.angular.x=np.nanstd(xac)
            pos_est.angular.y=np.nanstd(yac)
            pos_est.angular.z=np.nanstd(zac)
        else:
            pos_est.linear.x=-42
            pos_est.linear.y=-42
            pos_est.linear.z=-42
            pos_est.angular.x=0
            pos_est.angular.y=0
            pos_est.angular.z=0
        pubber.publish(pos_est)
        active_tags=[0,0,0,0]
        Position=[[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan],[np.nan,np.nan,np.nan]]
        rate.sleep()

if __name__=='__main__':
    try:
        pos_sub()
        pubsub()
    except rospy.ROSInterruptException:
        pass
