
import matplotlib.pyplot as plt
import math
 
g_longitudes = []
g_latitudes =[]

with open('2.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat = line.split()
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'r--')

g_longitudes = []
g_latitudes =[]

with open('debug.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat = line.split()
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'b--')


plt.show()
