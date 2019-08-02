
import matplotlib.pyplot as plt
import math
 
g_longitudes = []
g_latitudes =[]
steeringAngles = []
yaws = []

with open('1_1.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat,high,roll,yaw,pitch,n_vel,e_vel,u_vel,angleSpeedx,angleSpeedy,angleSpeedz,lineAccx,lineAccy,lineAccz,speed,steeringAngle= line.split(',')
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))
	steeringAngles.append(float(steeringAngle))
	yaws.append(float(yaw))


plt.plot(g_latitudes,g_longitudes,'r-*')
#plt.plot(range(len(steeringAngles)),steeringAngles)
#plt.plot(range(len(yaws)),yaws)
"""
g_longitudes = []
g_latitudes =[]

with open('debug.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat = line.split()
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'b--')

plt.savefig('a.pdf')
"""
plt.show()
