# nuitrack_core
The core package for interfacing with nuitrack library

## Subscribe

none


## Publish

* /nuitrack/rgb/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) : RGB image
* /nuitrack/depth/points ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) : PointCloud2 data
* /nuitrack/skeletons ([nuitrack_msgs/SkeletonDataArray](../nuitrack_msgs/msg/SkeletonDataArray.msg)) : User Skeleton Data
* /nuitrack/detected_users ([nuitrack_msgs/UserDataArray](../nuitrack_msgs/msg/UserDataArray.msg)) : User Detection Data
* /nuitrack/event/person_appeared ([nuitrack_msgs/EventUserUpdate](../nuitrack_msgs/msg/EventUserUpdate.msg)) : Event data for Person Appeared
* /nuitrack/event/person_disappeared ([nuitrack_msgs/EventUserUpdate](../nuitrack_msgs/msg/EventUserUpdate.msg)) : Event data for Person Disappeared