
import matplotlib.pyplot as plt
import math





def main():
	a = ('sss',12,18.52)
	b = ['aaa',1,1.1]
	with open('a.txt','w') as f:
		f.write('%s\t%d\t%f\n' %a)
		f.write('%s\t%d\t%f\n' %tuple(b))
	
	list1 = [1,2,3,4,5]
	list2 = ['a','b','c','d','e']
	
	list3 = [(num,alpha) for num in list1 for alpha in list2]
	
	print(list3)








if __name__ == '__main__':
	main()
