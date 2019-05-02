## Ephemeris Data for each satellite

## There are 33 ephemeris channels so that the element number will correspond to
##  the PRN number

Header header
    uint32 seq
    time stamp
    string frame_id

string  gps_id
float64 gps_time

# how far to iterate in.
int8 obs
int8[33] prn

int8[33]    health

# Positioning
float64[33] semimajor_axis    # sqrt(A), SQUARE ROOT of semi major axis (meters)
float64[33] mean_anomaly      # M0, Mean anomoly of reference time (rad)
float64[33] eccentricity      # ecc, 
float64[33] perigee_arg       # omega, Arguement of perigee (rad)
float64[33] cos_latitude      # cuc, Arguement of latitude, amplitude of cosine (rad)
float64[33] sin_latitude      # cus, Arguement of latitude, amplitude of sine (rad)
float64[33] cos_orbit_radius  # crc, Orbit radius, amplitude of cosine (rad)
float64[33] sin_orbit_radius  # crs, Orbit radius, amplitude of sine (rad)
float64[33] cos_inclination   # cic, Inclination, amplitude of cosine (rad)
float64[33] sin_inclination   # cis, Inclination, amplitude of sine (rad)
float64[33] inclination_angle # I0, Inclination angle at reference time (rad)
float64[33] right_ascension   # omega0,  Right ascension (rad)

# Movement
float64[33] mean_motion_diff  # Delta N, Mean motion difference (rad/s)
float64[33] inclination_rate  # I dot, Rate of inclination angle (rad/s)
float64[33] ascension_rate    # omega0 dot, Rate of right ascension (rad/s)

# Timing
float64[33] time_of_week      # tow, Time stamp of subframe 0 (seconds)
float64[33] reference_time    # toe, Reference time for ephemeris (seconds)
float64[33] clock_correction  # toc, SV clock correction term (seconds)
float64[33] group_delay       # tgd, Estimated group delay difference (seconds)
float64[33] clock_aging_1     # af0, Clock aging parameter (seconds)
float64[33] clock_aging_2     # af1, Clock aging parameter (seconds/seconds)
float64[33] clock_aging_3     # af2, Clock aging parameter (seconds/seconds/seconds)