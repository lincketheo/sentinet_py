#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
import math

class TagGrab:
    def __init__(self, pos_topic):
        self.pos_topic = pos_topic
        self.pospub = rospy.publisher(pos_topic,geometry_msgs.msg.Vector3, queue_size=1)
        self.bot_pos = geometry_msgs.msg.Vector3()
    
    rospy


if __name__=='__main__':
    rospy.init_node('Bot_Position', anonymous = True)
    t=tf2_ros.BufferCore(rospy.Duration(1))
    tfBuffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tfBuffer)

    tag_grab_one = TagGrab("pos_one")
    tag_grab_two = TagGrab("pos_two")
    tag_grab_three = TagGrab("pos_three")
    tag_grab_four = TagGrab("pos_four")

    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
