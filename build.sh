#!/bin/sh
if [ "$1" ];then
	if [ "$1" = "help" ];then
		cat README.TXT 
	else
		echo "unknown parameter.please type 'build.sh help' for help!"
	fi
else
	echo "now building robot! please type 'build.sh help' for help!"
	cd src
	make robot
	cp robot ../robot
	cd ..
fi