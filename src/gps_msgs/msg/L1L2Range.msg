## This msg created 5/30/2013
## originally intended for sending to DRTK or TDCP

Header header
    uint32 seq
    time stamp
    string frame_id

string gps_id
int64 gps_time

# number of observations to follow ( for debugging )
int32 num_obs

GpsRange L1
GpsRange L2

float64 lat
float64 lon
float64 alt
float64 lat_cov
float64 lon_cov
float64 alt_cov