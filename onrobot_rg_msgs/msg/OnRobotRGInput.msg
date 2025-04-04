# gFOF : Current fingertip offset in 1/10 millimeters. The value is a signed two's complement number.
uint16 g_fof

# gGWD : Current width between the gripper fingers in 1/10 millimeters.
#        The width is provided without any fingertip offset, as it is measured between the insides of the aluminum fingers.
uint16 g_gwd

# gSTA : Current device status, which indicates the status of the gripper and its motion.
# Bit       - Name              - Description
# 0 (LSB)   - Busy              - High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
# 1         - Grip detected     - High (1) when an internal- or external grip is detected.
# 2         - S1 pushed         - High (1) when safety switch 1 is pushed.
# 3         - S1 trigged        - High (1) when safety circuit 1 is activated. The gripper will not move while this flag is high.
# 4         - S2 pushed         - High (1) when safety switch 2 is pushed.
# 5         - S2 trigged        - High (1) when safety circuit 2 is activated. The gripper will not move while this flag is high.
# 6         - Safety error      - High (1) when on power on any of the safety switch is pushed.
# 7 - 15    - Reserved          - Not used.
uint16 g_sta

# gWDF : Current width between the gripper fingers in 1/10 millimeters.
#        The set fingertip offset is considered.
uint16 g_wdf

# All 4 status signals
uint16 sta_fing_l
uint16 sta_fing_r
uint16 sta_prox_l
uint16 sta_prox_r

# Signal that indicates if th gripper is busy (1) or accepts new commands (0)
uint16 busy

# Signal that indicates whether an external or internal grip is detected (1)
uint16 grip_det

# Proximity offsets of both fingers in 1/10 millimeters
uint16 prox_off_l
uint16 prox_off_r

# Force values along all 3 axis of the left finger in 1/10 newton. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.
int32 fx_l
int32 fy_l
int32 fz_l

# Torque values about all 3 axis of the left finger in 1/100 newton-meter. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.
int32 tx_l
int32 ty_l
int32 tz_l

# Force Values along all 3 axis of the right finger in 1/10 newton. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.
int32 fx_r
int32 fy_r
int32 fz_r

# Torque values about all 3 axis of the right finger in 1/100 newton-meter. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.
int32 tx_r
int32 ty_r
int32 tz_r

# Proximity values of both sensors in 1/10mm
int32 prox_l
int32 prox_r

# Actual gripper width without any offset in 1/10 millimeters
int32 grip_width

# Current state of the Bias, that sets force and torque to zero if set to 1
int8 in_zero
