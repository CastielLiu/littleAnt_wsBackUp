# Extended status message for RPV output
string gps_id_from
string gps_id_to

# GPS Time of output (ms into week)
int64 gps_time

int8 status # Output status
            #  2 : High precision solution available
            #  1 : Low precision solution only (high precision is zero)
            #  0 : Insufficient observations
            # -1 : Time stamps of input data do not match
            # -2 : Already computed solution for time stamp on input data
            # -3 : No L1 range measurements on receiver 1
            # -4 : No L2 range measurements on receiver 1
            # -5 : No L1 range measurements on receiver 2
            # -6 : No L2 range measurements on receiver 2

# ratio resultant from the LAMBDA method
float64 ratio

# number of input observations common across all with no cycle slip and which 
# are continuous between timesteps
int32 input_obs

# numb of 
int32 non_fault_obs

# Number of fixed ambiguiity observations
int32 amb_obs

## base prn used for double differencing - floating ambiguities
int32 base_prn_float

## base prn used for double differencing - fixed integer ambiguities
int32 base_prn_fixed