#!/usr/bin/env python3
"""
Publishes the world -> minithex_base transform as geometry_msgs/PoseStamped.
Looks up the full TF chain: world -> rslidar_imu -> rslidar -> minithex_base.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("tf_to_pose")

    parent_frame = rospy.get_param("~parent_frame", "world")
    child_frame  = rospy.get_param("~child_frame",  "minithex_base")
    rate_hz      = rospy.get_param("~rate",          100.0)

    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher("/tf_pose/world_to_minithex_base", PoseStamped, queue_size=10)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            tf_stamped = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))

            pose = PoseStamped()
            pose.header.stamp    = tf_stamped.header.stamp
            pose.header.frame_id = parent_frame

            pose.pose.position.x = tf_stamped.transform.translation.x
            pose.pose.position.y = tf_stamped.transform.translation.y
            pose.pose.position.z = tf_stamped.transform.translation.z

            pose.pose.orientation.x = tf_stamped.transform.rotation.x
            pose.pose.orientation.y = tf_stamped.transform.rotation.y
            pose.pose.orientation.z = tf_stamped.transform.rotation.z
            pose.pose.orientation.w = tf_stamped.transform.rotation.w

            pub.publish(pose)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        rate.sleep()


if __name__ == "__main__":
    main()
