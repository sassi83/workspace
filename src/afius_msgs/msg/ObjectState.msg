# Interface between FusionSystems modules and FgaF modules, see ObjectList.msg
# sub-msg of Object

# position (center of box)
float64      x
float64      y
float32      z

# object yaw rotation
float32      yaw

# object velocity
float32      velocity

# object course angle
float32      course_angle

# object course rate
float32      course_rate

# x, y, velocity, course angle, course rate (row-major representation of the
# lower triangular part)
float32[15]  covariance

# variance of z value
float32      z_variance