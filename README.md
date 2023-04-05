# filter_tf

Simple Utility Node to filter a specified ROS 2 TF transform

## Node `filter_tf`

### Parameters

* `child_frame` (string)

  This is the raw child frame we are observing. Ex: Table1, raw observations of transform from camera to table frame.

* `min_observation_count` (int, default: 10)

  The number of required observations before we begin filtering the transform.

* `destination_frame` (string, default: `child_frame` + `_filtered`)

  This is the name of the frame we will publish.

* `parent_frame` (string, default "world")

  This is the parent frame for the transform that we want to filter.

* `fraction_rotation` (double, default: 0.01)

  The fraction used in filtering the rotation.

* `fraction_translation` (double, default: 0.01)

  The fraction used in filtering the translation.

* `hz` (int, default: 10)

  This is the sampling frequency of the destination frame.
