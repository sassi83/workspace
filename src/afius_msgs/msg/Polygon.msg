# Interface between FusionSystems modules and FgaF modules, see PolygonList.msg
# Polygon that describes a specific section of the floor surrounding the bike

# polygon id
uint16                      id

# list of polygon points
geometry_msgs/Point[]       point_list

# enum sensor coverage and different free spaces (labeling guide)
afius_msgs/ObjectCategory   category