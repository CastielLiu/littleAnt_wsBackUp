## Useful general comment goes here
## Each receiver will be able to track differing numbers of satellites, so the
## fields are each of length 32, the number of SV's in the GPS constellation

# number of observations --> how far to iterate through the arrays
uint8 obs

## sv number for each satellite
uint8[33] prn

float64[33] psr
float32[33] psr_std

Carrier carrier