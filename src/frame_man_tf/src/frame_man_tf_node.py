#!/usr/bin/python

import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


def handle_pose(msg):
    """
    Transform `base_link` frame, in reference to `world` frame, with parameters specified in `msg`.
    """
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    rospy.Subscriber("/ekf_p", PoseStamped, handle_pose)
    rospy.spin()
