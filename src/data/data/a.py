
import matplotlib.pyplot as plt
import math

""" 
g_x = []
g_y =[]


with open('path.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	x,y,_ = line.split()
	g_x.append(float(x))
	g_y.append(float(y))


plt.plot(g_x,g_y,'b--')

"""
g_x = []
g_y =[]


with open('path.txt','r') as f:
	lines = f.readlines()
	
	for line in lines:
		x,y,_ = line.split()
		g_x.append(float(x))
		g_y.append(float(y))


plt.plot(g_x,g_y,'r--')

plt.plot(672098.2923,3529088.468162,'b*')

plt.plot(g_x[128],g_y[128],'k*')
plt.plot(g_x[129],g_y[129],'k*')

plt.axis('equal')
plt.grid('on')

#plt.savefig('a.pdf')

plt.show()
