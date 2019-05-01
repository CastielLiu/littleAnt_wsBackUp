
import matplotlib.pyplot as plt
import math


debug_x = []
debug_y = []


with open('debug.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	x,y,_ = line.split()
	debug_x.append(float(x))
	debug_y.append(float(y))


plt.plot(debug_x,debug_y,'b--')


g_x = []
g_y =[]


with open('path.txt','r') as f:
	lines = f.readlines()
	
	for line in lines:
		x,y,_ = line.split()
		g_x.append(float(x))
		g_y.append(float(y))


plt.plot(g_x,g_y,'r--')


plt.axis('equal')
plt.grid('on')

plt.savefig('a.pdf')

plt.show()
