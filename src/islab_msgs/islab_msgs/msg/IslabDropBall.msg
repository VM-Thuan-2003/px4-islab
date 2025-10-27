uint64 timestamp            # Message timestamp (e.g., in nanoseconds)
uint64 timestamp_sample     # Sampled data timestamp

# For 5 balls: use arrays of length 5
uint64[5] ball2drop         # Command: 1 to drop, 0 to hold for each ball
uint64[5] ball_id           # ID of each ball (e.g., [1,2,3,4,5])

bool[5] drop                # Drop status: True to drop, False otherwise for each ball

# Constants for coding logic
uint8 DROP     = 1          # Drop command value
uint8 NO_DROP  = 0          # No drop command value
