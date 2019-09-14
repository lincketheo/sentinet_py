#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
import math

if __name__=='__main__':
    rospy.init_node('Bot_Position',anonymous=True)
    t=tf2_ros.BufferCore(rospy.Duration(1))
    tfBuffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tfBuffer)

    pospub=rospy.Publisher('Bot_Pos0',geometry_msgs.msg.Vector3, queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            trans_front=tfBuffer.lookup_transform('camera','front',rospy.Time(),rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        timer=trans_front.header.stamp.secs

        if (rospy.get_time()-timer)<1.5:
            bot_pos=geometry_msgs.msg.Vector3()

            quat=[trans_front.transform.rotation.x,trans_front.transform.rotation.y,trans_front.transform.rotation.z,trans_front.transform.rotation.w]
            pitch,yaw,roll=euler_from_quaternion(quat)

            bot_pos.x=-trans_front.transform.translation.x-0.25
            bot_pos.y=4.7123+yaw #CCW angle off pos x from dump bucket corner, still needs stepper transform
            bot_pos.z=trans_front.transform.translation.z
            if math.fabs(roll)>2:
                bot_pos.x=-bot_pos.x
                bot_pos.y=1.5707-yaw
        else:
            bot_pos.y=-42

        pospub.publish(bot_pos)
        rate.sleep()
