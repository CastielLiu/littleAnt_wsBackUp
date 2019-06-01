1.  ./record_path.sh path.txt 
	[记录路径GPS坐标点 并生成路径曲率, path.txt 为文件名]
2.  roslaunch little_ant path_traking.launch  pathTracking_file_name:=path.txt  max_speed:="30.0"
	[启动路径跟踪,file_name 应与记录时的文件名一致， 其他参数查看launch文件]

