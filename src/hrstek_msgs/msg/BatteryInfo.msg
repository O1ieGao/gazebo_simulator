# battery type is in frame_id
std_msgs/Header header
float32 voltage 		# V
float32 current			# A
float32 capacity_remain 	# Ah
float32 capacity		# Ah
uint32 circle_count		# if unavailable, use 0
bool charging_status		# 1 means charging, 0 means not
bool charger_connected	# 1 means connected
float32 temperature		#, C, -1000, if unavailable
int8 percentage		# 0 to 100, means 0% to 100%, -1 means no bms info read.
uint8[] power_supply_health

