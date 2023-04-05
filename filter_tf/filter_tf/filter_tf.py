import math

import rclpy
from rclpy import qos
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Transform
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy
import numpy as np

_EPS = numpy.finfo(float).eps * 4.0

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. eucledian norm, along axis.
    >>> v0 = numpy.random.random(3)
    >>> v1 = unit_vector(v0)
    >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
    True
    >>> v0 = numpy.random.rand(5, 4, 3)
    >>> v1 = unit_vector(v0, axis=-1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = unit_vector(v0, axis=1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float64)
    >>> unit_vector(v0, axis=1, out=v1)
    >>> numpy.allclose(v1, v2)
    True
    >>> list(unit_vector([]))
    []
    >>> list(unit_vector([1.0]))
    [1.0]
    """
    if out is None:
        data = numpy.array(data, dtype=numpy.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(numpy.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = numpy.array(data, copy=False)
        data = out
    length = numpy.atleast_1d(numpy.sum(data*data, axis))
    numpy.sqrt(length, length)
    if axis is not None:
        length = numpy.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

def quaternion_slerp(quat0, quat1, fraction, spin=0, shortestpath=True):
    """Return spherical linear interpolation between two quaternions.
    >>> q0 = random_quaternion()
    >>> q1 = random_quaternion()
    >>> q = quaternion_slerp(q0, q1, 0.0)
    >>> numpy.allclose(q, q0)
    True
    >>> q = quaternion_slerp(q0, q1, 1.0, 1)
    >>> numpy.allclose(q, q1)
    True
    >>> q = quaternion_slerp(q0, q1, 0.5)
    >>> angle = math.acos(numpy.dot(q0, q))
    >>> numpy.allclose(2.0, math.acos(numpy.dot(q0, q1)) / angle) or \
        numpy.allclose(2.0, math.acos(-numpy.dot(q0, q1)) / angle)
    True
    """
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    if fraction == 0.0:
        return q0
    elif fraction == 1.0:
        return q1
    d = numpy.dot(q0, q1)
    if abs(abs(d) - 1.0) < _EPS:
        return q0
    if shortestpath and d < 0.0:
        # invert rotation
        d = -d
        q1 *= -1.0
    angle = math.acos(d) + spin * math.pi
    if abs(angle) < _EPS:
        return q0
    isin = 1.0 / math.sin(angle)
    q0 *= math.sin((1.0 - fraction) * angle) * isin
    q1 *= math.sin(fraction * angle) * isin
    q0 += q1
    return q0


class FilterTf(Node):

    def __init__(self):

        ########################################
        ## Initialize ROS node

        super().__init__("filter_tf_node")

        ########################################
        ## Get ROS parameters

        # The number of required observations before we begin
        # filtering the transform.
        self.declare_parameter('min_observation_count', 10)
        self.min_observation_count = int(self.get_parameter('min_observation_count').value)
        # This is the raw child frame we are observing. Ex: Table1,
        # raw observations of transform from camera to table frame
        self.declare_parameter('child_frame')
        self.observed_child_frame = str(self.get_parameter('child_frame').value)
        # This is the name of the frame we will publish
        self.declare_parameter('destination_frame', self.observed_child_frame+'_filtered')
        self.filtered_child_frame = str(self.get_parameter('destination_frame').value)
        # This is the parent frame for the transform that we want to
        # filter.
        self.declare_parameter('parent_frame', 'world')
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        # The fraction used in filtering the rotation.
        self.declare_parameter('fraction_rotation', 0.01)
        self.fraction_rotation = float(self.get_parameter('fraction_rotation').value)
        # The fraction used in filtering the translation.
        self.declare_parameter('fraction_translation', 0.01)
        self.fraction_translation = float(self.get_parameter('fraction_translation').value)
        # This is the sampling frequency of the destination frame
        self.declare_parameter('hz', 10)
        hz = int(self.get_parameter('hz').value)

        ########################################
        ## Setup tf2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)        

        ########################################
        ## Setup final variables

        # This is a moving average of the translational component
        # of the transform we are filtering
        self.filtered_trans = None
        # This is a moving average of the rotational component
        # of the transform we are filtering
        self.filtered_rot = None
        # number of times the raw transform has been observed
        self.observation_count = 0

        ########################################
        ## Start timer
        self.get_logger().info("Starting publish of transform")        
        dt = 1./float(hz)
        self.create_timer(dt, self.run)

    def _observe_raw_tf(self):
        raw_translation = None
        raw_rotation = None

        try:
            msg = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.observed_child_frame,
                rclpy.time.Time(),
            )
            raw_translation = [getattr(msg.transform.translation, d) for d in 'xyz']
            raw_rotation = [getattr(msg.transform.rotation, d) for d in 'xyzw']
            if self.observation_count < self.min_observation_count:
                self.observation_count += 1            
        except TransformException as err:
            self.get_logger().error("Failed to lookup transform for %s to %s" % (self.parent_frame, self.observed_child_frame))

        return raw_translation, raw_rotation

    def _update_filtered_tf(self, raw_translation, raw_rotation):
        # If this is the first time we have received a valid
        # transformation, define filtered_rot + filtered_trans
        if self.filtered_rot is None or self.observation_count < 20:
            self.filtered_rot = raw_rotation
        if self.filtered_trans is None or self.observation_count < 20:
            self.filtered_trans = np.array(raw_translation)

        # Actual filtering of the transformation
        # between the observed and published transform
        self.filtered_rot = quaternion_slerp(
            self.filtered_rot, raw_rotation, self.fraction_rotation)
        self.filtered_trans = (1.0-self.fraction_translation) * self.filtered_trans + self.fraction_translation * np.array(
            raw_translation)

    def _broadcast_filtered_tf(self):
        if self.filtered_trans is None or self.filtered_rot is None:
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        
        # get a new observation of the raw transform if available
        # returns None, None if tf timeout
        raw_translation, raw_rotation = self._observe_raw_tf()

        # update our filtered version of the transform
        if raw_translation and raw_rotation:
            self._update_filtered_tf(raw_translation, raw_rotation)

        # broadcast the filtered transform
        self._broadcast_filtered_tf()


def main(args=None):

    # Start node, and spin
    rclpy.init(args=args)
    node = FilterTf()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()    


if __name__ == '__main__':
    main()
