#!/usr/bin/python
#

# Given a pose (geometry_msgs/PoseStamped) between a world frame (the
# PoseStamped's frame_id) and a base frame (e.g. base_link), compute
# and publish a transform between the world frame and some ancestor of
# the base frame (e.g. odom) such that the transform from the world
# frame to the base frame is the pose given by the PoseStamped
# message.

import rospy
from tf import TransformListener, TransformBroadcaster, transformations, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped

class CorrectOdomNode:
    def __init__(self):
        # listen to tf messages to get transfrom from odom->base_link
        self.tfl = TransformListener()

        # broadcast world->odom transform
        self.tfb = TransformBroadcaster()

        # @todo: get frame ids as parameters
        self.base_frame_id = 'base_link'
        self.odom_frame_id = 'odom'
        # note: world frame id provided by the PoseStamped header
        
        self.sub = rospy.Subscriber("pose_stamped", PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        # base to odom (b2o) transform
        try:
            (b2o_trans, b2o_rot) = self.tfl.lookupTransform(self.base_frame_id, self.odom_frame_id,  rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logwarn_throttle(10, "tf exception in pose_callback()... ignoring")
            return
        b2o = transformations.concatenate_matrices(transformations.translation_matrix(b2o_trans),
                                                   transformations.quaternion_matrix(b2o_rot))

        # world to base (w2b) transform
        w2b_trans = transformations.translation_matrix((msg.pose.position.x,
                                                        msg.pose.position.y,
                                                        msg.pose.position.z))
        w2b_rot = transformations.quaternion_matrix((msg.pose.orientation.x,
                                                     msg.pose.orientation.y,
                                                     msg.pose.orientation.z,
                                                     msg.pose.orientation.w))
        w2b = transformations.concatenate_matrices(w2b_trans, w2b_rot)
                                                             
        # world to base * base to odom = world to odom (w2o)
        w2o = transformations.concatenate_matrices(w2b, b2o)

        # broadcast world to odom
        w2o_trans = transformations.translation_from_matrix(w2o)
        w2o_rot = transformations.quaternion_from_matrix(w2o)
        self.tfb.sendTransform(w2o_trans, w2o_rot, msg.header.stamp, self.odom_frame_id, msg.header.frame_id)

if __name__ == "__main__":
    rospy.init_node("correct_odom_node")
    node = CorrectOdomNode()
    rospy.spin()
