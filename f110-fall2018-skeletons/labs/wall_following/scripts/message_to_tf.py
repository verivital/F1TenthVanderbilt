#!/usr/bin/env python
import rospy 
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
import tf2_ros

br=tf.TransformBroadcaster()
br2=tf2_ros.TransformBroadcaster()
def vesc_odom_callback(data):
    global br
    a=Odometry()
    currentTime=data.header.stamp
    rospy.loginfo("currentTime: "+str(currentTime))
    frameID=data.header.frame_id
    rospy.loginfo("Frame ID: "+frameID)
    childFrame=data.child_frame_id
    rospy.loginfo("Child Frame ID: "+childFrame)
    pose=data.pose.pose
    rospy.loginfo("pose: "+str(pose))
    twist=data.twist.twist
    rospy.loginfo("twist: "+str(twist))

    odom_trans=TransformStamped()
    odom_trans.header.stamp=currentTime
    odom_trans.header.frame_id="odom"
    odom_trans.child_frame_id="base_link"

    odom_trans.transform.translation.x=pose.position.x
    odom_trans.transform.translation.y=pose.position.y
    odom_trans.transform.translation.z=pose.position.z
    odom_trans.transform.rotation=pose.orientation
    br.sendTransformMessage(odom_trans)

    tf1 = TransformStamped(
            header=Header(
                frame_id="map",
                stamp=currentTime
            ),
            child_frame_id="odom",
            transform=Transform(
                translation=pose.position,
                rotation=pose.orientation
            )
        )
    br2.sendTransform(tf1)






def vesc_odom_listener():
    rospy.init_node("message_to_tf",anonymous=True)
    rospy.Subscriber("/vesc/odom",Odometry,vesc_odom_callback)
    rospy.spin()

if __name__=="__main__":
    while not rospy.is_shutdown():
        vesc_odom_listener()
