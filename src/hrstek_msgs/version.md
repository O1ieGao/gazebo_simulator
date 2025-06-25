# Bito msgs版本说明

Updated by: Jin Dai

Date: 06/04/2020

## 1. bito_msgs介绍

bito_msgs是多条产品线/项目线共用的软件包，提供了自定义的消息数据结构。
在实际部署时，若几台设备的bito_msgs版本不兼容，则会在命令行中看到md5sum报错。

在实际的产品开发中，为了在代码中兼容多个版本的协议，需要在节点中，加入头文件，并在代码中使用BITO_MSGS_VERSION_MAJOR和BITO_MSGS_VERSION_MINOR

```cpp
#include "bito_msgsConfig.h"

#if BITO_MSGS_VERSION_MAJOR == 2

(code that's compatible with bito_msgs v2.0 only)

#endif

#if BITO_MSGS_VERSION_MAJOR >= 2

(code that's compatible with bito_msgs starting from v2.0)

#endif
```


## 2. 版本号规则

在产品开发中，若为了开发新的功能，核心的消息类型有升级，比如AgentMonitorASMsg、TaskSimplex2dMsg、BaseCommandMsg，导致无法向前兼容，则需要将major版本号加1。
若只是新增， 对已有的接口无修改，则将minor版本号加1。
在更新版本号时，需要修改CMakelist.txt中的。bito_msgs/include中的文件是自动生成的，修改无效。

## 3. 版本记录

------

### v1.0

该版本为2019年之前的初始版本，提供基本的消息数据结构。

------

### v2.0

为了适配任务提前下发、自动充电、轨迹顺滑切换等功能，产品化YG1.1、HX2.1新增并修改了数据结构，版本号升级为2.0.

- （新增）bito_msgsConfig.h头文件管理bito_msgs版本号
- （新增）TrajectoryPoint2dOptionMsg.msg
- （新增）Int8Vector.msg
- （新增）GeneralStatusMsg.msg
- （新增）BaseCommandMsg.msg
- （新增）RangeArray.msg
- （新增）CalibrationStatusMsg.msg
- （新增）CanMsg.msg
- （新增）CanMsgArray.msg
- （新增）GetGitInfoSrv.srv
- （新增）SteerWheelSrv.srv
- （新增）MushinyEmbeddedInterface.srv
- （新增）GetStringSrv.srv
- （新增）RoadmapInfoSrv.srv
- （删除）StampedTwistMsg.msg
- （修改）AgentMonitorASMsg.msg新增cargo_status
- （修改）AgentMonitorMsg.msg新增cpu_usage_percent、ram_usage_percent、temperature_zone、disk_usage_percent、lidar_status
- （修改）AgentStateListMsg.msg新增tc_remaining_time
- （修改）BatteryInfoMsg.msg中percentage数据类型从uint8变为int8，-1时代表bms读取异常
- （修改）ChargingStationStatusMsg.msg中修改了所有内容
- （修改）DetectionFieldMsg.msg中新增了header
- （修改）EmbeddedChargingControllerMsg.msg中，使用了n_stations，并将station数据结构改为数组
- （修改）LiftCmd.msg新增了header、priority、action，删除了cmd和duration
- （修改）LiftInfo.msg新增了lift_motor_idle、lift_task_status、lift_velocity、cargo_weight、error_code，删除了lift_motor_reached_position
- （修改）MotorDirectCtrl.msg删除了motor_speed，加入了header、joint_name、command_type、velocity、position
- （修改）Trajectory2dMsg.msg新增了current_trajectory_id、current_trajectory_seq、controller_type、destination、task_type
- （修改）TrajectoryPoint2dMsg.msg新增了option
- （修改）ChargingDoneStatusSrv.srv删除charging_station_id，新增charging_station_serial
- （修改）IntListSrv.srv新增了response中的error_code
- （修改）LiftSrv.srv新增了command
- （修改）LiftStatusSrv.srv将status类型从int32改为int8
- （修改）SendSerialSrv.srv新增response中的error_code
- （修改）SoftwarePauseSrv.srv新增response中的error_code
- （修改）StartChargingSrv.srv删除了factory_station_id，新增了serial、pose、command、robot_serial，在response中新增了error_code和serial
- （修改）SwitchChargingStatusSrv.srv删除了factory_station_id，新增了charging_station_serial、status
- （修改）SwitchStateSrv.srv新增了宏定义TO_ZERO_VEL、SOFT_ESTOP、RESUME_SOFT_ESTOP、SLOW_SOFT_ESTOP

------

### v3.0

为了适配死锁检测、电梯、避障图区切换等功能，产品化YG1.2、HX2.2新增并修改了数据结构，版本号升级为3.0.

- （新增）DeadlockStateMsg.msg
- （新增）DeadlockStateAllMsg.msg
- （新增）RoiModeMsg.msg
- （新增）CheckTargetRegionObstacleSrv.srv
- （新增）RequestEnterEvelatorSrv.srv
- （新增）SetSensorTFSrv.srv
- （修改）AgentMonitorASMsg.msg新增current_floor和current_map
- （修改）TrajectoryPoint2dOptionMsg.msg新增floor

------

### v3.1

为了适配gpm_agf的交握和单机与韩信通信。v3.0新增加和修改了数据结构，版本号升级为3.1.

- （新增）Int32Vector.msg
- （新增）StampedInt32Vector.msg
- （新增）StampedInt32Vectors.msg
- （修改）AgentMonitorASMsg.msg新增了odometer