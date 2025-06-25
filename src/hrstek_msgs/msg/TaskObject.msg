## header, time stamp
std_msgs/Header header

## Task Object ID number
std_msgs/Int32 id

## Task Object operation type, add or delete or inquiry, etc.
## std_msgs/Int32 operation

## Task Sender Node Id, who has a task to be executed
std_msgs/Int32 nodeId 

## Task type, what type of task does the sender have
## A machine node will require picking up
## A truck node will require loading
std_msgs/Int32 taskType 

## Item type
std_msgs/Int32 itemTypeId

## Item quantitiy
std_msgs/Int32 itemNum

## other info, in case we need it. 
## for example, move cargo to transportation area, this field indicates types of cargo
std_msgs/Int32 other 
