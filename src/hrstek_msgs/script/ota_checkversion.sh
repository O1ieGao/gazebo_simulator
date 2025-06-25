set -e
current="$(cd "$(dirname "${BASH_SOURCE[0]}")"&&pwd)" 
home="/home/${HOSTNAME}"
function CHECK_VERSION(){
	str_branch=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "branch"|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	str_time=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "commit_date"|awk '{$1=$2=$7=""}1'|awk -F ' ' '{$5=$4;$4=$3;$3=$2;$2=$1;$1=$5;$5=""}1'|awk -F ' ' '{$1="_"$1;$2="_"$2;$3="_"$3;$4="_"$4}1'|sed s/[[:space:]]//g`
	str_time=${str_time//:/_}
	str_commit=`cat $home/$1/install/share/bito_msgs/version/version.json|grep "\"commit\""|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	update_commit=`cat ${current}/bito_common_share/bito_msgs/version/version.json|grep "\"commit\""|awk -F ': "' '{print $2}'|awk -F '"' '{print $1}'`
	echo "update_commit"
	echo ${update_commit}
	if [ $str_commit == $update_commit ]; then
	    echo "Current version is the same as the installation package!" 
	    exit 1
	fi
	str_commit=${str_commit:0:4}  #first four commit numbers
	if [ -d "${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0}" ]; then
	    echo "${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0} ALREADY EXISTS" 
	    rm -rf ${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0}
	fi
	if [ -d "${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz" ]; then
	    echo "${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz ALREADY EXISTS" 
	    rm -rf ${current}/../rollback/bito_common_${str_branch:0}_${str_commit:0}${str_time:0}.tar.gz
	fi
	file_name="bito_common_"${str_branch:0}_${str_commit:0}${str_time:0}
}
function MAIN(){
	if [ -d "${home}/hanxin_ws" ];then
		echo "check hanxin bito_common version"
		file_name="hanxin_ws"
		CHECK_VERSION ${file_name}
	else
		echo "hanxin_ws 文件夹不存在"
	fi
	if [ -d "${home}/yugong_ws" ];then
		echo "check yugong bito_common version"
		file_name="yugong_ws"
		CHECK_VERSION ${file_name}
	else
		echo "yugong_ws 文件夹不存在"
	fi
}
MAIN