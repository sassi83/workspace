# Interface between FusionSystems modules and FgaF modules, see ObjectList.msg

# Describes stops per tour

# timestamp
builtin_interfaces/Time      stamp

# unique identifier object
uint64                        id

# center of gravity
float32                       x_centroid
float32                       y_centroid
float32                       z_centroid

# bounding box
float32                       length
float32                       width
float32                       height

# Information about the state of the object
afius_msgs/ObjectState        state

# the most likely object categories
afius_msgs/ObjectCategory[]   category_list

# of points (TODO: which?)
float32[6]                    covariance