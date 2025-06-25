# number of samples
uint8 num_samples

# calibration status
# 0 - idle, no data being collected, user can adjust parameters
# 1 - collecting data
# 2 - data enough for calibration, will keep adding
# 3 - data validation, user can cherrypick samples
# 4 - calculating
# 5 - calculation finished, show results
# 6 - data saved
uint8 IDLE=0
uint8 COLLECTING=1
uint8 READY_FOR_CALIBRATION=2
uint8 DATA_VALIDATION=3
uint8 CALCULATING=4
uint8 CALCULATION_FINISHED=5
uint8 DATA_SAVED=6
uint8 calibration_status

