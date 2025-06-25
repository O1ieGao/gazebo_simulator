
string[] serial

  # ASID_Online = 0,
  # ASID_Offline = 1,
  # ASID_Registered = 2,
  # ASID_Waiting = 3, // agent only
  # ASID_Sending = 4, // server only
  # ASID_Permitting = 5,
  # ASID_RunTaskTraj = 6,
  # ASID_DoneTaskTraj = 7,
  # ASID_RunAction = 8,
  # ASID_DoneAction = 9,
  # ASID_RunHomingTraj = 10,
  # ASID_DoneHomingTraj = 11,
  # ASID_Exception = 12,
  # ASID_PermitAction = 13,
  # ASID_SendingAction = 14,
  # ASID_TotalNumber
int32[] state_id

string[] state_name

string[] dispatching_state

int32[] pre_notice

int32[] suspend

string[] suspend_source

int32[] action_state_block

int64[] current_task_complex_id

uint32[] tc_remaining_time

int32[] current_node_id
# unknown = 0,
# idle = 1,
# busy = 2
# go waiting area = 3
int32[] task_state
