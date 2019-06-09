#!/bin/bash

source devel/setup.bash

basedir=`pwd`

file_path="$basedir/src/data/data/raw/"

if [ -z "$1" ]; then
	echo "please input file name  *.txt"
	exit
else
	file_name="$1"
fi

roslaunch little_ant record_gps_data.launch file_path:="$file_path"  file_name:="$file_name"

python src/data/data/tool/generate_curvature.py src/data/data/raw/$file_name src/data/data/result/$file_name

echo "generate result path information completed."

