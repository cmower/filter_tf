#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped

spike_probability = 0.9

def eps(n, lo, hi):
    eps = np.random.uniform(lo, hi, size=n)
    if np.random.uniform(0, 1) < spike_probability:
        return 8*eps
    else:
        return eps

def pos():
    p = np.ones(3) + eps(3, -0.005, 0.005)
    t = rospy.Time.now().to_sec()
    p1_offset = np.sin(t * 2.0 * np.pi * 0.5) * 0.1
    p2_offset = np.sin(t * np.pi * 0.5) * 0.2
    p[1] += p1_offset
    p[2] += p2_offset
    return p

def quat():
    q = np.array([0, 0, 0, 1]) + eps(4, -0.005, 0.005)
    q /= np.linalg.norm(q)
    return q

def dt():
    return 0.01 + eps(1, 0, 0.01)[0]

def main():
    rospy.init_node('test_filter_tf_node')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():

        # Generate pose
        p = pos()
        q = quat()

        # Broadcast transform
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'test_tf'
        msg.transform.translation.x = p[0]
        msg.transform.translation.y = p[1]
        msg.transform.translation.z = p[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        tf_broadcaster.sendTransform(msg)

        # Sleep
        rospy.sleep(dt())

if __name__ == '__main__':
    main()
