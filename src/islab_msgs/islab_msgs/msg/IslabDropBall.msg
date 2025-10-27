# DropBallArray.msg

uint64 timestamp            # Message timestamp (e.g., in nanoseconds)
uint64 timestamp_sample     # Sampled data timestamp

uint64[5] ball_id           # IDs for 5 balls

bool[5] drop                # Drop status for 5 balls

uint8 DROP     = 1          # Drop command value
uint8 NO_DROP  = 0          # No drop command value
