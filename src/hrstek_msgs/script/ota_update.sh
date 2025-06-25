set -e
current="$(cd "$(dirname "${BASH_SOURCE[0]}")"&&pwd)" 
home="/home/${HOSTNAME}"
function DELETE(){
	echo $2
	$2 rm -rf $1/share/bito_msgs
	$2 rm -rf $1/share/bito_interface
	$2 rm -rf $1/share/bito_obdb
	$2 rm -rf $1/share/common-lisp/ros/bito_msgs
	$2 rm -rf $1/share/gennodejs/ros/bito_msgs
	$2 rm -rf $1/share/roseus/ros/bito_msgs
	$2 rm -rf $1/include/bito_interface
	$2 rm -rf $1/include/bito_msgs
	$2 rm -rf $1/include/bito_obdb
	$2 rm -rf $1/lib/python2.7/dist-packages/bito_msgs
}

function UPDATE(){
	$2 cp -r ${current}/bito_common_share/bito_* $1/share
	$2 cp -r ${current}/bito_common_share/common-lisp/ros/bito_msgs $1/share/common-lisp/ros/
	$2 cp -r ${current}/bito_common_share/gennodejs/ros/bito_msgs $1/share/gennodejs/ros/
	$2 cp -r ${current}/bito_common_share/roseus/ros/bito_msgs $1/share/roseus/ros/
	$2 cp -r ${current}/bito_common_include/* $1/include
	$2 cp -r ${current}/bito_common_lib/python2.7/dist-packages/bito_msgs $1/lib/python2.7/dist-packages
}

function MAIN(){
	if [ -d "${home}/hanxin_ws/install" ];then
		echo "check hanxin bito_common version"
		file_name="${home}/hanxin_ws/install"
		DELETE ${file_name}
		UPDATE ${file_name}
	else
		echo "hanxin_ws/install 文件夹不存在"
	fi
	if [ -d "${home}/yugong_ws/install" ];then
		echo "check yugong bito_common version"
		file_name="${home}/yugong_ws/install"
		DELETE ${file_name}
		UPDATE ${file_name}
	else
		echo "yugong_ws/install文件夹不存在"
	fi
	if [ -d "/opt/bslam/" ];then
		echo "update slam bito_common code"
		file_name="/opt/bslam"
		command="sudo"
		DELETE ${file_name} ${command}
		UPDATE ${file_name} ${command}
	else
		echo "slam 文件夹不存在"
	fi
}
MAIN
