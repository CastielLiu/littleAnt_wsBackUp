
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
			x,y,yaw,curvature = line.split()
			self.x.append(float(x))
			self.y.append(float(y))
			self.yaw.append(float(yaw))
			self.curvature.append(float(curvature))
		return True
			
	def clear(self):
		self.x.clear()
		self.y.clear()
		self.yaw.clear()
		self.curvature.clear()
		
	def calculateCurvature(self):
		self.curvature = [0.0]*len(self.x)
		for i in range(len(self.x)-1):
			theta = self.yaw[i+1] - self.yaw[i]
			dis = self.__disBetweenPoints(i,i+1)
			if(dis==0):
				self.curvature[i] = 0.0
			else:
				self.curvature[i] = 2*math.sin(theta/2)/dis
				#self.curvature[i] = theta/dis
			if(self.curvature[i]>0.3):
				self.curvature[i] = 0.3;
			elif(self.curvature[i]<-0.3):
				self.curvature[i] = -0.3
		self.curvature[len(self.x)-1] = 0.0
	def curvatureFilter(self,count):
		num = int(count)/2
		cur = [0.0] * len(self.curvature)
		for i in range(len(self.curvature)-1)[num:-num]:
			temp_list = self.curvature[num+i:num+i+count]
			cur[i] = sum(temp_list)/count
		self.curvature = cur[:]
		
	def offsetPoints(self,x,y):
		self.x = [X-x for X in self.x]
		self.y = [Y-y for Y in self.y]
		
	def dump(self,file_name):
		with open(file_name,'w') as f:
			for i in range(len(self.x)):
				f.write('%.3f\t%.3f\t%.3f\t%.5f\n' %(self.x[i],self.y[i],self.yaw[i],self.curvature[i]))
				
	def __disBetweenPoints(self,i,j):
		x = self.x[i] - self.x[j]
		y = self.y[i] - self.y[j]
		return math.sqrt(x*x+y*y)
		

			
def plot(file_name):
	path_points = Points()
	path_points.load(file_name)
	reference_point_x = path_points.x[0]
	reference_point_y = path_points.y[0]
	
	path_points.offsetPoints(reference_point_x,reference_point_y)
	
	plt.plot(path_points.y,path_points.x,'r.',label="reference path")
	index = 4882
	plt.plot(path_points.y[index],path_points.x[index],'b*')
	index = 4997
	plt.plot(path_points.y[index],path_points.x[index],'b*')
	plt.axis('equal')

	plt.legend()

	#plt.ylim((0,30))
	
	#plt.xticks(np.arange(0,200,2))
	plt.grid('on')

	#plt.savefig('a.pdf')

	plt.show()

if(len(sys.argv) != 2):
	print("please input file name")
else:
	plot(sys.argv[1])
