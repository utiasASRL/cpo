uint64 t_a                      # time a in nanoseconds (UTC)
uint64 t_b                      # time b in nanoseconds (UTC)
SatPair[] pairs                 # set of msgs to construct TDCP factors
geometry_msgs/Vector3 enu_pos       # approximate position in the local East North Up frame at time b
geometry_msgs/Vector3 prev_enu_pos  # previous calculated approximate position (typically at time a)
geometry_msgs/Vector3 enu_origin    # geodetic position (lat long alt) [deg/deg/m] of local ENU frame