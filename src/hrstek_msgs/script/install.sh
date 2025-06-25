#!/bin/bash
set -e
# 需进入到当前目录执行此脚本

# 获取此脚本所在目录的名称，即安装包名称
pack_name=$(basename "$PWD")

path=`pwd`

function backup() {
    echo "start backup"
    # 进入目标资源所在的目录
    bash ${path}/$1/ota_rollback.sh
}
function COMPRESS_ROLLBACK(){
    tar -czvf ${pack_name}.tar.gz rollback
    echo "backup ok"
}

function install() {
    echo "start install"
    bash ${path}/$1/ota_update.sh
}

function success() {
    # 最终完成需要输出success(这里强制要求需要输出，作为成功的标识)
    echo "success"
}

function main() {
	SRC=`find -type f | ls *.gz`
	for file in $SRC; do
	  tar -xzvf $file
	  filename=`basename $file .tar.gz`
      backup ${filename}
      install ${filename}
      rm -rf ${filename}
	done
    # backup
    # install
    success
}

main
