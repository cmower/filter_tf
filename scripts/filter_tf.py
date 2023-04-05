#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf_conversions
from geometry_msgs.msg import TransformStamped

"""Provide a filtered version of a desired transform. Ex. we have raw
observations of Table1 frame in the camera frame of reference, this
node will produce a transform from camera_frame to Table1_filtered."""


class TfFilter:
    def __init__(self):
        ########################################
        ## Initialize ROS node

        rospy.init_node("filter_tf_node")

        ########################################
        ## Get ROS parameters

        # The number of required observations before we begin
        # filtering the transform.
        self.min_observation_count = rospy.get_param("~min_observation_count", 10)
        # This is the raw child frame we are observing. Ex: Table1,
        # raw observations of transform from camera to table frame
        self.observed_child_frame = rospy.get_param("~child_frame")
        # This is the name of the frame we will publish
        self.filtered_child_frame = rospy.get_param(
            "~destination_frame", self.observed_child_frame + "_filtered"
        )
        # This is the parent frame for the transform that we want to
        # filter.
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        # The fraction used in filtering the rotation.
        self.fraction_rotation = rospy.get_param("~fraction_rotation", 0.01)
        # The fraction used in filtering the translation.
        self.fraction_translation = rospy.get_param("~fraction_translation", 0.01)
        # This is the sampling frequency of the destination frame
        hz = rospy.get_param("~hz", 10)

        ########################################
        ## Setup tf2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        ########################################
        ## Setup final variables

        # This is a moving average of the translational component
        # of the transform we are filtering
        self.filtered_trans = None
        # This is a moving average of the rotational component
        # of the transform we are filtering
        self.filtered_rot = None
        # This is the rate that our tranform will be published at
        self.rate = rospy.Rate(hz)
        # number of times the raw transform has been observed
        self.observation_count = 0

    def _observe_raw_tf(self):
        raw_translation = None
        raw_rotation = None
        try:
            msg = self.tf_buffer.lookup_transform(
                self.parent_frame, self.observed_child_frame, rospy.Time()
            )
            raw_translation = [getattr(msg.transform.translation, d) for d in "xyz"]
            raw_rotation = [getattr(msg.transform.rotation, d) for d in "xyzw"]
            if self.observation_count < self.min_observation_count:
                self.observation_count += 1
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(
                "Failed to lookup transform for %s to %s"
                % (self.parent_frame, self.observed_child_frame)
            )

        return raw_translation, raw_rotation

    def _update_filtered_tf(self, raw_translation, raw_rotation):
        # If this is the first time we have received a valid
        # transformation, define filtered_rot + filtered_trans
        if (
            self.filtered_rot is None
            or self.observation_count < self.min_observation_count
        ):
            self.filtered_rot = raw_rotation
        if (
            self.filtered_trans is None
            or self.observation_count < self.min_observation_count
        ):
            self.filtered_trans = np.array(raw_translation)

        # Actual filtering of the transformation
        # between the observed and published transform
        self.filtered_rot = tf_conversions.transformations.quaternion_slerp(
            self.filtered_rot, raw_rotation, self.fraction_rotation
        )
        self.filtered_trans = (
            1.0 - self.fraction_translation
        ) * self.filtered_trans + self.fraction_translation * np.array(raw_translation)

    def _broadcast_filtered_tf(self):
        if self.filtered_trans is None or self.filtered_rot is None:
            return

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.filtered_child_frame
        msg.transform.translation.x = self.filtered_trans[0]
        msg.transform.translation.y = self.filtered_trans[1]
        msg.transform.translation.z = self.filtered_trans[2]
        msg.transform.rotation.x = self.filtered_rot[0]
        msg.transform.rotation.y = self.filtered_rot[1]
        msg.transform.rotation.z = self.filtered_rot[2]
        msg.transform.rotation.w = self.filtered_rot[3]
        self.tf_broadcaster.sendTransform(msg)

    def run(self):
        rospy.loginfo("Starting publish of transform")

        while not rospy.is_shutdown():
            # get a new observation of the raw transform if available
            # returns None, None if tf timeout
            raw_translation, raw_rotation = self._observe_raw_tf()

            # update our filtered version of the transform
            if raw_translation and raw_rotation:
                self._update_filtered_tf(raw_translation, raw_rotation)

            # broadcast the filtered transform
            self._broadcast_filtered_tf()

            self.rate.sleep()


if __name__ == "__main__":
    filter = TfFilter()
    filter.run()
