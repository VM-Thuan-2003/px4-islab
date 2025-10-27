uint64 timestamp
uint64 timestamp_sample

uint64[5] ball_id
bool[5] dropped
bool[5] in_progress
uint8[5] status_code
string[5] message

uint8 STATUS_WAITING    = 0
uint8 STATUS_DROPPING   = 1
uint8 STATUS_DROPPED    = 2
uint8 STATUS_ERROR      = 255
