#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry


def odom_callback(msg, br):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    br.sendTransform(
        (pos.x, pos.y, pos.z),
        (ori.x, ori.y, ori.z, ori.w),
        rospy.Time.now(),
        msg.child_frame_id or "minithex_base",
        msg.header.frame_id or "world",
    )


if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback, callback_args=br)
    rospy.spin()
