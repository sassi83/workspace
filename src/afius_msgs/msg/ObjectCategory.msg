# Interface between FusionSystems modules and FgaF modules, see ObjectList.msg

# submsg of Object

# classification category for the object
uint8  category

# (0 .. 100) [%]
int8  category_probability

# Enum values for members "category"

# Values for semantic segmentation
uint8 EGO_VEHICLE     = 0
uint8 BICYCLE_LANE    = 1
uint8 PEDESTRIAN_PATH = 2
uint8 SHARED_BICYCLE_PEDESTRIAN_PATH = 3
uint8 CAR_LANE        = 4
uint8 SEALED_FREE_SPACE = 5
uint8 UNSEALED_PATH   = 6
uint8 HORIZONTAL_VEGETATION = 7
uint8 PARKING_SPACE   = 8
uint8 RAILWAY_TRACK   = 9
uint8 CURB_STONE      = 10
uint8 CENTRE_LINE     = 11
uint8 LANE_BOUNDARY_STRIPS = 12
uint8 VERTICAL_BARRIER = 13
uint8 VEHICLE         = 14
uint8 CYCLIST_WITH_BICYCLE_SS = 15
uint8 BICYCLE_WITHOUT_CYCLIST_SS = 16
uint8 MOTORCYCLIST_WITH_MOTORCYCLE_SS = 17
uint8 MOTORCYCLE_WITHOUT_MOTORCYCLIST_SS = 18
uint8 PEDESTRIAN      = 19
uint8 TRAFFIC_SIGN    = 20
uint8 TRAFFIC_LIGHT   = 21
uint8 VERTICAL_VEGETATION = 22
uint8 SKY             = 23
uint8 MISCELLANEOUS_STATIC_OBJECT = 24
uint8 MISCELLANEOUS_DYNAMIC_OBJECT = 25

# Values for bounding box detection
uint8 SPEED_BUMP      = 26
uint8 CAR             = 27
uint8 BUS             = 28
uint8 TRAM            = 29
uint8 TRANSPORTER     = 30
uint8 TRUCK           = 31
uint8 CYCLIST_WITH_BICYCLE_BB = 32
uint8 BICYCLE_WITHOUT_CYCLIST_BB = 33
uint8 MOTORCYCLIST_WITH_MOTORCYCLE_BB = 34
uint8 MOTORCYCLE_WITHOUT_MOTORCYCLIST_BB = 35
uint8 AMBULANCE_VEHICLE = 36
uint8 POLICE_VEHICLE  = 37
uint8 FIRE_TRUCK      = 38
uint8 TRAFFIC_LIGHT_CATEGORY = 39
uint8 ADULT_WITHOUT_BABY_BUGGY_OR_MOBILITY_SUPPORT = 42
uint8 CHILD_WITHOUT_BABY_BUGGY_OR_MOBILITY_SUPPORT = 43
uint8 PERSON_WITH_BABY_BUGGY = 44
uint8 PERSON_WITH_MOBILITY_SUPPORT = 45
uint8 DOG             = 46
uint8 SIGNALLING_DEVICE_RAILROAD_CROSSING = 47
uint8 TRAFFICMARKING_FUSSGAENGERUEBERWEG = 84
uint8 TRAFFICMARKING_RADWEG = 85
uint8 TRAFFICMARKING_GEHWEG = 86
uint8 TRAFFICMARKING_GEMEINSAMER_GEH_UND_RADWEG = 87
uint8 TRAFFICMARKING_GETRENNTER_RAD_UND_GEHWEG_RADWEG_LINKS = 88
uint8 TRAFFICMARKING_GETRENNTER_RAD_UND_GEHWEG_RADWEG_RECHTS = 89
uint8 TRAFFICMARKING_HALTLINIE = 90
uint8 BIKE            = 33
uint8 CONTAINER       = 201
uint8 STATIC_OBSTACLE = 202

# Meta classes of bounding box detection
uint8 BIG_ROAD_VEHICLE = 150
uint8 CYCLIST_WITH_BIKE = 151
uint8 BIKE_WITHOUT_CYCLIST = 152
uint8 PERSON = 153