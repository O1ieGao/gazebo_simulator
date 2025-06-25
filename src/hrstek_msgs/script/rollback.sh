# backup installations
function install(){
    . ./$1/ota_update.sh
}
function main() {
	echo "start rollback"
    SRC=`find -type f | ls *.gz`
    for file in $SRC; do
        tar -xzvf $file
        filename=`basename $file .tar.gz`
        install ${filename}
        rm -rf ${filename}
	done
	echo "rollback ok"
}
main
