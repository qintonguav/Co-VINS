Header header
int16 seq
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Point32[] point_3d
geometry_msgs/Point32[] point_2d
int64[] point_des
geometry_msgs/Point32[] feature_2d
int64[] feature_des


# points refer to window features, which contains 3D postion
# feature refer to fast feature in whole image
# points_2d can be removed in the future

# to do list: distance resample
