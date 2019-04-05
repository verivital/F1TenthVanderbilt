#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('base_link_to_laser')
    br = tf.TransformBroadcaster()
    br2=tf2_ros.TransformBroadcaster()
    #rate = rospy.Rate(10000.0)


    #Here I want to publish the transform between the odom frame and the map since we fix the odom frame to the map 
    #frame all the transforms are 0
    a=Odometry()
    while not rospy.is_shutdown():
        br.sendTransform((0.265, 0.0, 0.157),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")
    	# pose2=a.pose.pose.position
    	# #("pose"+str(pose2))
    	# pose2.x=0
    	# pose2.y=0
    	# pose2.z=0
    	# print("pose: "+str(pose2))
    	# tf1 = TransformStamped(
        #     header=Header(
        #         frame_id="odom",
        #         stamp=rospy.Time.now()
        #     ),
        #     child_frame_id="base_link",
        #     transform=Transform(
        #         translation=pose2,
        #         rotation=Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        #     )
        # )
        # br2.sendTransform(tf1)
