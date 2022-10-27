# Interface between FusionSystems modules and FgaF modules,
# from Freespace Adaption (FS)
# to prediction (FgaF)

# List of Polygons

# The polygons describe the nature of the ground that touches and surrounds the bike.

# timestamp and id referend to lidar data
std_msgs/Header      scan_header

# List of polygons
afius_msgs/Polygon[] polygon_list
