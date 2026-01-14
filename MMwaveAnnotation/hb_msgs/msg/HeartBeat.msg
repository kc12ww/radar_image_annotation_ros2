# heart beat message of a sensor node, publishes every 200ms

# name of the sensor
string name

# time stamp
builtin_interfaces/Time stamp

# error code, should be zero for normal heart beat messages
int32 code

# description of the code
string reason
