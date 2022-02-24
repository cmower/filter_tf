# filter_tf

Simple Utility Node to filter a specified ROS TF transform

## Node `filter_tf.py`

### Parameters

* `~child_frame` (string)

  This is the raw child frame we are observing. Ex: Table1, raw observations of transform from camera to table frame.

* `~min_observation_count` (int, default: 10)

  The number of required observations before we begin filtering the transform.

* `~destination_frame` (string, default: `~child_frame` + `_filtered`)

  This is the name of the frame we will publish.

* `~parent_frame` (string, default "world")

  This is the parent frame for the transform that we want to filter.

* `~fraction_rotation` (double, default: 0.01)

  The fraction used in filtering the rotation.

* `~fraction_translation` (double, default: 0.01)

  The fraction used in filtering the translation.

* `~hz` (int, default: 10)

  This is the sampling frequency of the destination frame.


## Args

child_frame: raw observation of a reference frame ex: /ar_marker_8

parent_frame: frame in which the observation occurs. ex: /kinect2_link

This node will then publish a tf from parent_frame to child_frame_filtered. ex: /ar_marker_8_filtered

## Usage:

From command line:
```
rosrun filter_tf filter_tf.py _child_frame:=CHILD_FRAME_ID [other parameters]
```
In launch file:
```xml
<node pkg="filter_tf" name="filter" type="filter_tf.py">
  <param name="child_frame" type="string" value="CHILD_FRAME_ID"/>
</node>
```

This will now broadcast a transform from `world` to `CHILD_FRAME_ID_filtered`. This transform will continue to broadcast even if the raw transform becomes occluded.  This transform will be updated via a moving average whenever a new observation of the raw transform occurs.
