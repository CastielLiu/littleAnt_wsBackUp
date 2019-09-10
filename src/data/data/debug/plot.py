
import matplotlib.pyplot as plt
import numpy as np
import math
import sys


def main(argv):
	file_name = argv[1]
	with open(file_name,'r') as f:
		lines = f.readlines()
	if lines is None:
		return False
		
	for line in lines:
		x,y,yaw,curvature = line.split()
		self.x.append(float(x))
		self.y.append(float(y))
		self.yaw.append(float(yaw))
		self.curvature.append(float(curvature))	
		
	

			
def plot(file_name):
	path_points = Points()
	file_name = "../result/" + file_name
	path_points.load(file_name)
	
	reference_point_x = path_points.x[0]
	reference_point_y = path_points.y[0]
	
	path_points.offsetPoints(reference_point_x,reference_point_y)
	
	result_points = Points()
	file_name = "../raw/" + file_name
	result_points.load(file_name)
	
	result_points.offsetPoints(reference_point_x,reference_point_y)
	
	plt.plot(path_points.y,path_points.x,'r.',label="reference path")
	plt.plot(result_points.y,result_points.x,'k-',label="trajectory")

	plt.legend()

	#plt.ylim((0,30))
	
	#plt.xticks(np.arange(0,200,2))
	plt.grid('on')

	plt.savefig('a.pdf')

	plt.show()

if(len(sys.argv) != 2):
	print("please input file name")
else:
	plot(sys.argv[1])
