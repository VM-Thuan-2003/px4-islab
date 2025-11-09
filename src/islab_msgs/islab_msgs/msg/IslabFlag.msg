
# IslabFlagAuto.msg

uint64 timestamp           # Time since system start (microseconds)
uint64 timestamp_sample    # Timestamp when data was sampled (microseconds)

bool flag_auto              # Flag indicating automatic mode
bool flag_start_auto        # Flag to start automatic mode
bool flag_stop_auto         # Flag to stop automatic mode
bool flag_manual            # Flag indicating manual mode
bool flag_start_manual      # Flag to start manual mode
bool flag_stop_manual       # Flag to stop manual mode