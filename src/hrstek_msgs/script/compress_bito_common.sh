#!/bin/bash
set -e
home="/home/${HOSTNAME}"
function COMPRESS_FILE(){
	# get git info
	str_branch=`cat ~/$1/install/share/bito_msgs/version/version.json|grep "branch"|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	str_commit=`cat ~/$1/install/share/bito_msgs/version/version.json|grep "\"commit\""|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	str_commit=${str_commit:0:4}  #first four commit numbers
	str_time=`cat ~/$1/install/share/bito_msgs/version/version.json|grep "commit_date"|awk '{$1=$2=$7=""}1'|awk -F ' ' '{$5=$4;$4=$3;$3=$2;$2=$1;$1=$5;$5=""}1'|awk -F ' ' '{$1="_"$1;$2="_"$2;$3="_"$3;$4="_"$4}1'|sed s/[[:space:]]//g`
	str_time=${str_time//:/_}

	# create backup folders
	cd $home
	folder_name="bito_common_"${str_branch:0}_${str_commit:0}${str_time:0}
	mkdir -p ${folder_name}/bito_common_include
	mkdir -p ${folder_name}/bito_common_lib/python2.7/dist-packages
	mkdir -p ${folder_name}/bito_common_share/gennodejs/ros
	mkdir -p ${folder_name}/bito_common_share/common-lisp/ros
	mkdir -p ${folder_name}/bito_common_share/roseus/ros



	# backup installations
	cp -r ${home}/$1/install/share/bito_msgs ${folder_name}/bito_common_share
	cp -r ${home}/$1/install/share/bito_interface ${folder_name}/bito_common_share
	cp -r ${home}/$1/install/share/bito_obdb ${folder_name}/bito_common_share
	cp -r ${home}/$1/install/share/common-lisp/ros/bito_msgs ${folder_name}/bito_common_share/common-lisp/ros/
	cp -r ${home}/$1/install/share/gennodejs/ros/bito_msgs ${folder_name}/bito_common_share/gennodejs/ros/
	cp -r ${home}/$1/install/share/roseus/ros/bito_msgs ${folder_name}/bito_common_share/roseus/ros/

	cp -r ${home}/$1/install/include/bito_interface ${folder_name}/bito_common_include
	cp -r ${home}/$1/install/include/bito_msgs ${folder_name}/bito_common_include
	cp -r ${home}/$1/install/include/bito_obdb ${folder_name}/bito_common_include

	cp -r ${home}/$1/install/lib/python2.7/dist-packages/bito_msgs ${folder_name}/bito_common_lib/python2.7/dist-packages

	cp -r ${home}/$1/install/share/bito_msgs/script/update.sh ${folder_name}
	cp -r ${home}/$1/install/share/bito_msgs/script/ota_update.sh ${folder_name}
	cp -r ${home}/$1/install/share/bito_msgs/script/ota_rollback.sh ${folder_name}
	cp -r ${home}/$1/install/share/bito_msgs/script/ota_checkversion.sh ${folder_name}

	chmod 777 ${folder_name}/update.sh ${folder_name}/ota_update.sh ${folder_name}/ota_rollback.sh ${folder_name}/ota_checkversion.sh
	# zip -q -r -P bitorobotics ${folder_name}.zip ${folder_name}
	tar zcvf ${folder_name}.tar.gz ${folder_name}
	rm -rf ${folder_name}
}

function MAIN(){
	if [ -d "${home}/hanxin_ws" ];then
		file_name="hanxin_ws"
		COMPRESS_FILE ${file_name}
	else
		echo "hanxin_ws 文件夹不存在"
	fi
	if [ -d "${home}/yugong_ws" ];then
		file_name="yugong_ws"
		COMPRESS_FILE ${file_name}
	else
		echo "yugong_ws 文件夹不存在"
	fi
}
MAIN
echo "SUCCESS"
