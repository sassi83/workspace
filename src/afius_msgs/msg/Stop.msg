# Interface between ILM modules and IMS, see StopList.msg

# Describes stop gnss position (lat,long,alt)

# timestamp
builtin_interfaces/Time      stamp

# unique identifier object
uint64                        id

# position gnss
float32                       latitude
float32                       longitude
float32                       altitude

