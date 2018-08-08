# nuitrack_user_viewer
The visualization package for nuitrack_ros


## Subscribe

* /nuitrack/rgb/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) : RGB image
* /nuitrack/detected_users ([nuitrack_msgs/UserDataArray](../nuitrack_msgs/msg/UserDataArray.msg)) : User Detection Data
* /nuitrack/skeletons ([nuitrack_msgs/SkeletonDataArray](../nuitrack_msgs/msg/SkeletonDataArray.msg)) : User Skeleton Data


## Publish

* /nuitrack/viz_user_markers ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)) : The marker of box visualization for current position of users
* /nuitrack/viz_skeleton_markers ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)) : The marker of skeleton visualization for current users


## Visualization

        $ rviz

    Append item MarkerArray and select topics