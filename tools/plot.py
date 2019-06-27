
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import re
		


def main(argv):
	if(len(argv)<2):
		print("please input file name")
		return
		
	Datas = []
	with open(argv[1],'r') as f:
		line = f.readline()  ##removethe first line 
		while True:
			line = f.readline()
			if(line==""):
				break
			Data = re.split('[,;]+',line)
			Data = [float(data) for data in Data]
			Datas.append(Data)
	Datas = np.array(Datas)
	
	fig1 = plt.figure("gps")
	plt.subplot(211)
	gps_ax1 = fig1.gca()
	gps_ax1.plot(Datas[:,0],Datas[:,1],'r*')
	plt.subplot(212)
	gps_ax2 = fig1.gca()
	gps_ax2.plot(range(Datas.shape[0]),Datas[:,4],'b')
	
	fig2 = plt.figure("vehicle")
	plt.subplot(211)
	vehicle_ax1 = fig2.gca()
	vehicle_ax1.plot(range(Datas.shape[0]),Datas[:,15],label="speed")
	plt.subplot(212)
	vehicle_ax2 = fig2.gca()
	vehicle_ax2.plot(range(Datas.shape[0]),Datas[:,16],label="steeringAngle")
	
	fig3 = plt.figure("imu")
	plt.subplot(311)
	imu_ax1 = fig3.gca()
	imu_ax1.plot(range(Datas.shape[0]),Datas[:,9],label="angleSpeedx")
	plt.subplot(312)
	imu_ax2 = fig3.gca()
	imu_ax2.plot(range(Datas.shape[0]),Datas[:,10],label="angleSpeedy")
	plt.subplot(313)
	imu_ax3 = fig3.gca()
	imu_ax3.plot(range(Datas.shape[0]),Datas[:,11],label="angleSpeedz")
	
	plt.show()
	
if __name__=="__main__":
	main(sys.argv)		
