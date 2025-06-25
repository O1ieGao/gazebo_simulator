
#ifndef BITO_MSGS_SRC_BITO_ENUM_HPP_
#define BITO_MSGS_SRC_BITO_ENUM_HPP_


namespace bito{

	/* task object related definition */
	enum TaskObjectOperationType : int{
		TASK_OBJ_OP_ADD = 0, // add a task
		TASK_OBJ_OP_DEL, // del a task
		TAKS_OBJ_OP_INQ // inquiry about a task
	};

	enum TaskObjectStatusType : int{
		TASK_OBJ_STATUS_RECV = 0, // recv a task
		TASK_OBJ_STATUS_ASSIGNED = 1, // task assigned
		TASK_OBJ_STATUS_DONE = 2, // task finished
		TASK_OBJ_STATUS_CANCELLED // task cancelled
	};

	enum TaskObjectTypeType: int{
		TASK_OBJ_TYPE_BLANK = 0, // do nothing
		TASK_OBJ_TYPE_PICK = 1, // pick from machine area
		TASK_OBJ_TYPE_PLACE = 2, // place n meters ahead from a node
		TASK_OBJ_TYPE_LOAD = 3, // pick from storageing area
		TASK_OBJ_TYPE_STORE = 4, // save to storaging area
		TASK_OBJ_TYPE_EXITG = 5, // exit the graph
		TASK_OBJ_TYPE_ENTERG = 6, // enter the graph
		TASK_OBJ_TYPE_LOOP = 7 // send to storage, pick from storage
	};

	/* robot executive related definitions */
	enum ExecActType : int{
		EXEC_ACT_REST = 0, // do nothing, just wait
		EXEC_ACT_PICK = 1, // simple pick
		EXEC_ACT_PLACE = 2, // simple place
		EXEC_ACT_TOSTORE = 3, // request storage manager
		EXEC_ACT_TOSTORE_R = 4,// request storage manager to retrieve
		EXEC_ACT_TOAREA = 5, // request area manager
		EXEC_ACT_TOAREA_R = 6, // request area manager to retrieve
		EXEC_ACT_TOTRANS = 7, // request transport manager
		EXEC_ACT_TOTRANS_R = 8, // request transport manager
		EXEC_ACT_TOTRANSAREA = 9, // request transarea manager
		EXEC_ACT_TOTRANSAREA_R = 10, // request transarea manager to retrieve, no use now
		// EXEC_ACT_TRANS_TYPEA, // transport cargo type A from storage to truck
		// EXEC_ACT_TRANS_TYPEB, // transport cargo type B from storage to truck
		// EXEC_ACT_TOTRAN, // request transportation area manager
		EXEC_ACT_PICK_S = 11, // pick in storaging area, a diff task from EXEC_ACT_PICK !
		EXEC_ACT_PLACE_S // place in storaging area, a diff task from EXEC_ACT_PLACE !
		//...
	};

	enum ExecStatus : int{
		EXEC_STATUS_INIT = 0, // constructor finished
		EXEC_STATUS_WRONG, // exec meet error, eg time stamp too old
		EXEC_STATUS_HUMAN, // require operator's help
		EXEC_STATUS_GLOCA, // first global localization finished
		EXEC_STATUS_ONGRAPH, // robot is now on a graph vertex
		EXEC_STATUS_READY, // exec is now ready for next action request
		// EXEC_STATUS_RUN,
		EXEC_STATUS_RECV, // exec received a task, this is newly added after using service
		EXEC_STATUS_TRAJ, // exec is now running for current action request
		EXEC_STATUS_PICK,
		EXEC_STATUS_PLACE,
		EXEC_STATUS_PICK_S,
		EXEC_STATUS_PLACE_S,
		EXEC_STATUS_TRAJ_S, // pick or place in storage area is done and go back to traj
		EXEC_STATUS_TRAJ_I // initialize call to trajectorizer to get onto graph
	};

	/* robot trajectorizer related definitions */
	enum TrajActType : int{
		TRAJ_ACT_BLANK = 0, // do nothing, execute next path segment
		TRAJ_ACT_STUN, // return to exec by calling a exec srv 
		TRAJ_ACT_STUNNED
	};

	enum TrajStatus : int{
		TRAJ_STATUS_INIT = 0,
		TRAJ_STATUS_RUN
	};

	/* picking server related definitions */
	enum PickStatus: int{
		PICK_STATUS_INIT = 0, // constructor finished
		PICK_STATUS_RLOCA, // robot localized
		PICK_STATUS_PLOCA, // pallet localized
		PICK_STATUS_PICK, // execute task pick
		PICK_STATUS_PICK_S, // execute task pick in storage area
		PICK_STATUS_PLACE, // execute task place, no use now
		PICK_STATUS_PLACE_S // execute task place in storage area
	};


};



#endif
