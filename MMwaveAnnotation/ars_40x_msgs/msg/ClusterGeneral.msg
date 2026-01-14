# can id 0x701 message

# cluster id
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
# moving or stationary, ignored in srr308 (0: moving, 1: stationary, 2: oncoming, 3: stationary candidate, 4: unknown, 5: crossing stationary, 6: crossing moving, 7: stopped)
uint32 dyn_prop
