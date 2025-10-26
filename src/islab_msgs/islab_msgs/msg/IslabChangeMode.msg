# IslabChangeModeMsg.msg

uint64 timestamp
uint64 timestamp_sample

uint64 mode
uint64 STABILIZEMODE = 0
uint64 ALTMODE = 1
uint64 POSHOLDMODE = 2
uint64 AUTOMODE = 3
uint64 OFFBOARDMODE = 4
uint64 RTHMODE = 5
uint64 LANDMODE = 6

float64 altitude
uint64 ALTITUDE_DEFAULT = 2

uint64 arm
bool ARMED = true
bool DISARMED = false

uint64 handel
uint64 HANDEL_MODE = 0
uint64 HANDEL_TAKEOFF = 1
uint64 HANDEL_ARM = 2

uint64 source
uint64 GROUNDSOURCE = 0
uint64 WEBSOURCE = 1
uint64 USERSOURCE = 2