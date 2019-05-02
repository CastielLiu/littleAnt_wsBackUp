# Message output by TDCP

Header header
    uint32 seq
    time stamp
    string frame_id

string  gps_id
float64 gps_time0
float64 gps_time1

float64 dt

## Status
##      0: Fault, standard positioning solution reported
##      1: All good
uint8 status

XYZRpvData xyz
ENURpvData enu
