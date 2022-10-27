# Interface between FusionSystems modules and FgaF modules,
# from Object Fusion and Prediciton (FS)
# to almost all FgaF modules, e.g. local_planner, prediction, ...

# Lists with all detacted objects

# (former World.msg)

# Everything that could potentially move belongs in the dynamic_object_list,
# even if it is not moving at the time of the observation, like parked cars,
# sitting and standing people, stopped public transport. Everything that will
# not move, like trees and traffic lights, belongs in the static list. So, that
# the downstream prediction module doesn’t need to parse two lists or one list
# with too many entries.

# see std_msgs/Header.msg
std_msgs/Header      header

# List of fused static objects
afius_msgs/Object[]  static_object_list

# List of fused dynamic objects
afius_msgs/Object[]  dynamic_object_list
