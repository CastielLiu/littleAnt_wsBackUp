
import matplotlib.pyplot as plt
import math
 
g_longitudes = []
g_latitudes =[]

with open('gps_first.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat = line.split()
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))

fig = plt.figure()	

fig.add_subplot(211)

plt.plot(g_latitudes,g_longitudes,'*')

plt.xlim(min(g_latitudes),max(g_latitudes))

#plt.axis('equal')

x=0
y=0
a = [0]*10
curves = [0]*10
for i in range(10,len(g_longitudes)-10):
	x = g_longitudes[i-10]+g_longitudes[i-9]+g_longitudes[i-8]+g_longitudes[i-7]+g_longitudes[i-6] + \
		g_longitudes[i-5]+g_longitudes[i-4]+g_longitudes[i-3]+g_longitudes[i-2]+g_longitudes[i-1] + \
		g_longitudes[i+5]+g_longitudes[i+4]+g_longitudes[i+3]+g_longitudes[i+2]+g_longitudes[i+1] - g_longitudes[i]*20 + \
		g_longitudes[i+6]+g_longitudes[i+7]+g_longitudes[i+8]+g_longitudes[i+9]+g_longitudes[i+10]
	y = g_latitudes[i-10]+g_latitudes[i-9]+g_latitudes[i-8]+g_latitudes[i-7]+g_latitudes[i-6] + \
		g_latitudes[i-5]+g_latitudes[i-4]+g_latitudes[i-3]+g_latitudes[i-2]+g_latitudes[i-1] + \
		g_latitudes[i+5]+g_latitudes[i+4]+g_latitudes[i+3]+g_latitudes[i+2]+g_latitudes[i+1] - g_latitudes[i]*20 + \
		g_latitudes[i+6]+g_latitudes[i+7]+g_latitudes[i+8]+g_latitudes[i+9]+g_latitudes[i+10]
		 
	curve = math.sqrt(x*x+y*y)
	print(curve)
	curves.append(curve)
	
curves+= a
print(max(curves))
fig.add_subplot(212)

print(len(curves),len(g_latitudes))
plt.plot(g_latitudes,curves);	
plt.xlim(min(g_latitudes),max(g_latitudes))
plt.show()
