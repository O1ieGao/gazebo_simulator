#!/bin/bash

set -o errexit

home="/home/${HOSTNAME}"
if [ -f ${home}/yugong_ws/src/bito_common/bito_msgs/script/getMachineCode.py ];then
  machine_code=`python ${home}/yugong_ws/src/bito_common/bito_msgs/script/getMachineCode.py`
elif [ -f ${home}/yugong_ws/install/share/bito_msgs/script/getMachineCode.py ];then
  machine_code=`python ${home}/yugong_ws/install/share/bito_msgs/script/getMachineCode.py`
elif [ -f /opt/bslam/share/bito_msgs/script/getMachineCode.py ];then
  machine_code=`python /opt/bslam/share/bito_msgs/script/getMachineCode.py`
elif [ -f ${home}/yugong_ws/install/share/bito_msgs/script/getMachineCode.py ];then
  machine_code=`python ${home}/hanxin_ws/src/bito_common/bito_msgs/script/getMachineCode.py`
elif [ -f ${home}/yugong_ws/install/share/bito_msgs/script/getMachineCode.py ];then
  machine_code=`python ${home}/hanxin_ws/install/share/bito_msgs/script/getMachineCode.py`
else
  echo "can not find machine_code"
  exit 1
fi

echo ${machine_code}




