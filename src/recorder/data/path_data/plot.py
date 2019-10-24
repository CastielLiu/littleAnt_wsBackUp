
import matplotlib.pyplot as plt
import numpy as np
import math
import sys


class Points:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []
		self.curvature = []
		
		
	def load(self,file_name):
		with open(file_name,'r') as f:
			lines = f.readlines()
		
		if lines is None:
			return False
			
		for line in lines:
			msg = line.split()
			self.x.append(float(msg[0]))
			self.y.append(float(msg[1]))
			self.yaw.append(float(msg[2]))
		return True
			
	def clear(self):
		self.x.clear()
		self.y.clear()
		self.yaw.clear()
		self.curvature.clear()
		
		
	def offsetPoints(self,x,y):
		self.x = [X-x for X in self.x]
		self.y = [Y-y for Y in self.y]
		
	def __disBetweenPoints(self,i,j):
		x = self.x[i] - self.x[j]
		y = self.y[i] - self.y[j]
		return math.sqrt(x*x+y*y)
		
			
def plot(file_names):
	i = 0
	for file_name in file_names:
		path_points = Points()
		path_points.load(file_name)
		if(i==0):
			plt.plot(path_points.y,path_points.x,'r.')
			i = i+1
		elif(i==1):
			plt.plot(path_points.y,path_points.x,'b.')
			i = i+1
		elif(i==2):
			plt.plot(path_points.y,path_points.x,'k.')
			
	index = 7150
	plt.plot(path_points.y[index],path_points.x[index],'b*')
	index = 7300
	plt.plot(path_points.y[index],path_points.x[index],'b*')
	plt.axis('equal')
	plt.legend()
	#plt.ylim((0,30))
	#plt.xticks(np.arange(0,200,2))
	plt.grid('on')

	#plt.savefig('a.pdf')

	plt.show()

if(len(sys.argv) < 2):
	print("please input file name")
else:
	plot(sys.argv[1:])
