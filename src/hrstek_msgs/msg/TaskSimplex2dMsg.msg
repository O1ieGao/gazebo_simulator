
# id of this task simplex
int64 id

# sequence number of this simplex, (id, seq) together forms a unique 
# identifier of each task simplex sent
int64 seq

# type is the action type
int32 type

# temporarily added for different height storage.
# When we do systematic change from 2d data structure to 3d data structure, 
# this field will be deleted since height will be contained in destination field.
float32 height

int32 task_status

int32 priority

TrajectoryPoint2dMsg destination

# TODO delete home field
TrajectoryPoint2dMsg home

string name

Trajectory2dMsg task_trajectory

Trajectory2dMsg servo_trajectory

# TODO delete homing_trajectory field
Trajectory2dMsg homing_trajectory

bool trajectory_continue
#send Trajectory TS in advance, it will be true
bool cached

bool servo_done

string other
