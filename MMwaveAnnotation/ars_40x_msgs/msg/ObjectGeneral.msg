# can id 0x60B message

# object id
uint32 id
# position x, in m
float64 dist_long
# position y, in m
float64 dist_lat
# relative velocity x, in m/s
float64 vrel_long
# relative velocity y, in m/s
float64 vrel_lat
# radar cross section, RCS, in dBm^2
float64 rcs
# object moving or stationary (0: moving, 1: stationary, 2: oncoming, 3: stationary candidate, 4: unknown, 5: crossing stationary, 6: crossing moving, 7: stopped)
int32 dyn_prop
