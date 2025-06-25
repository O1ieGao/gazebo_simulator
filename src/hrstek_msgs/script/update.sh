#!/bin/bash
set -e
current="$(cd "$(dirname "${BASH_SOURCE[0]}")"&&pwd)" 
home="/home/${HOSTNAME}"
function UPDATE_CODE(){
	#current=$PWD
	str_branch=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "branch"|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	str_time=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "commit_date"|awk '{$1=$2=$7=""}1'|awk -F ' ' '{$5=$4;$4=$3;$3=$2;$2=$1;$1=$5;$5=""}1'|awk -F ' ' '{$1="_"$1;$2="_"$2;$3="_"$3;$4="_"$4}1'|sed s/[[:space:]]//g`
	str_time=${str_time//:/_}
	str_commit=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "\"commit\""|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	update_commit=`cat ${current}/bito_common_share/bito_msgs/version/version.json|grep "\"commit\""|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	if [ $str_commit == $update_commit ]; then
	    echo "Current version is the same as the installation package!" 
	    exit 1
	fi
	str_commit=${str_commit:0:4}  #first four commit numbers
	if [ -d "${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0}" ]; then
	    echo "${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0} ALREADY EXISTS" 
	    rm -rf ${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0}
	fi
	if [ -d "${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz" ]; then
	    echo "${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz ALREADY EXISTS" 
	    rm -rf ${current}/../bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz
	fi
	cd ${current}/..
	folder_name="bito_common_"${str_branch:0}_${str_commit:0}${str_time:0}
	mkdir -p ${folder_name}/bito_common_include
	mkdir -p ${folder_name}/bito_common_lib/python2.7/dist-packages
	mkdir -p ${folder_name}/bito_common_share/gennodejs/ros
	mkdir -p ${folder_name}/bito_common_share/common-lisp/ros
	mkdir -p ${folder_name}/bito_common_share/roseus/ros
	cp -r ${current}/update.sh ./${folder_name}
	cp -r ${current}/ota_rollback.sh ./${folder_name}
	cp -r ${current}/ota_checkversion.sh ./${folder_name}
	cp -r ${current}/ota_update.sh ./${folder_name}
	chmod 777 ./${folder_name}/update.sh ./${folder_name}/ota_update.sh ./${folder_name}/ota_rollback.sh ./${folder_name}/ota_checkversion.sh

    # backup installations
    mv ${home}/$1/install/share/bito_msgs ${folder_name}/bito_common_share
	mv ${home}/$1/install/share/bito_interface ${folder_name}/bito_common_share
	mv ${home}/$1/install/share/bito_obdb ${folder_name}/bito_common_share
	mv ${home}/$1/install/share/common-lisp/ros/bito_msgs ${folder_name}/bito_common_share/common-lisp/ros/
	mv ${home}/$1/install/share/gennodejs/ros/bito_msgs ${folder_name}/bito_common_share/gennodejs/ros/
	mv ${home}/$1/install/share/roseus/ros/bito_msgs ${folder_name}/bito_common_share/roseus/ros/

	mv ${home}/$1/install/include/bito_interface ${folder_name}/bito_common_include
	mv ${home}/$1/install/include/bito_msgs ${folder_name}/bito_common_include
	mv ${home}/$1/install/include/bito_obdb ${folder_name}/bito_common_include

	mv ${home}/$1/install/lib/python2.7/dist-packages/bito_msgs ${folder_name}/bito_common_lib/python2.7/dist-packages

	# zip -q -r -P bitorobotics ${folder_name}.zip ${folder_name}
	tar zcvf ${folder_name}.tar.gz ${folder_name}
	rm -rf ${folder_name}
	cp -r ${current}/bito_common_share/bito_* ${home}/$1/install/share
	cp -r ${current}/bito_common_share/common-lisp/ros/bito_msgs ${home}/$1/install/share/common-lisp/ros/
	cp -r ${current}/bito_common_share/gennodejs/ros/bito_msgs ${home}/$1/install/share/gennodejs/ros/
	cp -r ${current}/bito_common_share/roseus/ros/bito_msgs ${home}/$1/install/share/roseus/ros/
	cp -r ${current}/bito_common_include/* ${home}/$1/install/include

	cp -r ${current}/bito_common_lib/python2.7/dist-packages/bito_msgs ${home}/$1/install/lib/python2.7/dist-packages
}
function UPDATE_slam_CODE(){
	sudo rm -rf /opt/bslam/share/bito_msgs
	sudo rm -rf /opt/bslam/share/bito_obdb
	sudo rm -rf /opt/bslam/share/bito_interface
	sudo rm -rf /opt/bslam/share/common-lisp/ros/bito_msgs
	sudo rm -rf /opt/bslam/share/gennodejs/ros/bito_msgs
	sudo rm -rf /opt/bslam/share/roseus/ros/bito_msgs
	sudo rm -rf /opt/bslam/include/bito_obdb
	sudo rm -rf /opt/bslam/include/bito_msgs
	sudo rm -rf /opt/bslam/include/bito_interface
	sudo rm -rf /opt/bslam/lib/python2.7/dist-packages/bito_msgs
	sudo cp -r  ${current}/bito_common_share/bito_* /opt/bslam/share
	sudo cp -r  ${current}/bito_common_share/common-lisp/ros/* /opt/bslam/share/common-lisp/ros/
	sudo cp -r  ${current}/bito_common_share/gennodejs/ros/* /opt/bslam/share/gennodejs/ros/
	sudo cp -r  ${current}/bito_common_share/roseus/ros/* /opt/bslam/share/roseus/ros/
	sudo cp -r  ${current}/bito_common_include/* /opt/bslam/include
	sudo cp -r  ${current}/bito_common_lib/python2.7/dist-packages/* /opt/bslam/lib/python2.7/dist-packages
}
function MAIN(){
	if [ -d "${home}/hanxin_ws" ];then
		echo "update hanxin bito_common code"
		file_name="hanxin_ws"
		UPDATE_CODE ${file_name}
	else
		echo "hanxin_ws 文件夹不存在"
	fi
	if [ -d "${home}/yugong_ws" ];then
		echo "update yugong bito_common code"
		file_name="yugong_ws"
		UPDATE_CODE ${file_name}
	else
		echo "yugong_ws 文件夹不存在"
	fi
	if [ -d "/opt/bslam/" ];then
		echo "update slam bito_common code"
		UPDATE_slam_CODE
	else
		echo "slam 文件夹不存在"
	fi
}
MAIN
echo "SUCCESS" 
