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

    tfBuffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tfBuffer)

    pospub=rospy.Publisher('Bot_Pos3',geometry_msgs.msg.Vector3, queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            trans_mid_side=tfBuffer.lookup_transform('camera','mid_side',rospy.Time(),rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        timer=trans_mid_side.header.stamp.secs

        if (rospy.get_time()-timer)<1.5:
            bot_pos=geometry_msgs.msg.Vector3()

            quat=[trans_mid_side.transform.rotation.x,trans_mid_side.transform.rotation.y,trans_mid_side.transform.rotation.z,trans_mid_side.transform.rotation.w]
            pitch,yaw,roll=euler_from_quaternion(quat)

            bot_pos.x=trans_mid_side.transform.translation.z
            bot_pos.y=yaw #CCW angle off pos x from dump bucket corner, still needs stepper transform
            bot_pos.z=trans_mid_side.transform.translation.x-0.825
            if math.fabs(roll)>2:
                bot_pos.z=-bot_pos.z
                bot_pos.y=-yaw
        else:
            bot_pos.y=-42

        pospub.publish(bot_pos)
        rate.sleep()
