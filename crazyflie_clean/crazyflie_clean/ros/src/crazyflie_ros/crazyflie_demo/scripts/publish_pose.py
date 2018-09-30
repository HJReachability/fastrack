#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    msg2 = PoseStamped()
    msg2.header.seq = 0
    msg2.header.stamp = rospy.Time.now()
    msg2.header.frame_id = worldFrame
    msg2.pose.position.x = x+.5
    msg2.pose.position.y = y
    msg2.pose.position.z = z
    msg2.pose.orientation.x = quaternion[0]
    msg2.pose.orientation.y = quaternion[1]
    msg2.pose.orientation.z = quaternion[2]
    msg2.pose.orientation.w = quaternion[3]

    msg3 = PoseStamped()
    msg3.header.seq = 0
    msg3.header.stamp = rospy.Time.now()
    msg3.header.frame_id = worldFrame
    msg3.pose.position.x = x+.5
    msg3.pose.position.y = y+.5
    msg3.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg3.pose.orientation.x = quaternion[0]
    msg3.pose.orientation.y = quaternion[1]
    msg3.pose.orientation.z = quaternion[2]
    msg3.pose.orientation.w = quaternion[3]
    msg4 = PoseStamped()
    msg4.header.seq = 0
    msg4.header.stamp = rospy.Time.now()
    msg4.header.frame_id = worldFrame
    msg4.pose.position.x = x
    msg4.pose.position.y = y+.5
    msg4.pose.position.z = z
    msg4.pose.orientation.x = quaternion[0]
    msg4.pose.orientation.y = quaternion[1]
    msg4.pose.orientation.z = quaternion[2]
    msg4.pose.orientation.w = quaternion[3]



    pub = rospy.Publisher(name, PoseStamped, queue_size=1)
    seq = 0
    flag= 0
    lasttime=rospy.get_time()
    while not rospy.is_shutdown():
        seq=seq+1
        if flag == 0:
            msg.header.seq = seq
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
        elif flag == 1:
            msg2.header.seq = seq
            msg2.header.stamp = rospy.Time.now()
            pub.publish(msg2)
        elif flag == 2:
            msg3.header.seq = seq
            msg3.header.stamp = rospy.Time.now()
            pub.publish(msg3)
        elif flag == 3:
            msg4.header.seq = seq
            msg4.header.stamp = rospy.Time.now()
            pub.publish(msg4)
        #if rospy.get_time()-lasttime>5:
        #    flag=(flag+1)%4
        #    lasttime=rospy.get_time()
        rate.sleep()

