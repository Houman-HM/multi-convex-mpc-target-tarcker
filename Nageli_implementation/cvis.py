import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def cvis(x, y, xm, ym, xo, yo, ao, bo):#(x, y, z, xm, ym, zm, xo, yo, zo, ao, bo, co):
	
	rt_T_rt = (xo - x) ** 2 + (yo - y) ** 2# + (zo - z) ** 2
	rh_T_rh = (xm - x) ** 2 + (ym - y) ** 2# + (zm - z) ** 2

	proj = (xm - x) * (xo - x) + (ym - y) * (yo - y)# + (zm - z) * (zo - z)
	
	condition_1 = proj - rt_T_rt + 1;
	condition_2 = proj / (rh_T_rh)# *  np.sqrt(rt_T_rt))

	A = xm - x
	B = ym - y
	
	A_ = A / ao
	B_ = B / bo

	X = x - xo
	Y = y - yo

	X_ = X / ao
	Y_ = Y / bo

	D = (2 * (A_*X_ + B_*Y_)) ** 2 - 4 * (A_**2 + B_**2) * (X_**2 + Y_**2 - 1)
	
	cost = np.zeros(x.shape)
	
	cost = ((abs(condition_1)/(condition_1 + 0.00000000001)) + 1) * (abs(condition_2)/(condition_2 + 0.000000000001) + 1) * condition_2
	return cost
	for i in range(len(cost)):
		for j in range(len(cost[0])):
			if condition_1[i][j] > 0 and condition_2[i][j] > 0:# and D[i][j] >= 0:
				cost[i][j] = condition_2[i][j]#proj[i][j] / (rh_T_rh[i][j])# * np.sqrt(rt_T_rt[i][j]))
	#beta = 30	
	#cost = (np.log(1 + np.exp(beta * condition_2)) / beta) * (np.log(1 + np.exp(beta * condition_1)) / beta)**2
	return cost


if __name__ == '__main__':

	xm = 5.0
	ym = 0.0
	#zm = 25.0
	
	xo = 3.0
	yo = 3.0
	#zo = 25.0;
	
	ao = 0.35
	bo = 0.35
	#co = 7

	x = np.linspace(-8, 8, 100)
	y = np.linspace(-6.0, 6, 100)
	#z = np.linspace(0.0, 50.0, 100)

	X, Y = np.meshgrid(x, y)
	value = cvis(X, Y, xm, ym, xo, yo, ao, bo)
	

	phi = np.linspace(0, 2*np.pi)
	theta = np.linspace(-np.pi/2, np.pi/2)
	phi, theta=np.meshgrid(phi, theta)

	ex = xo + np.cos(theta) * np.sin(phi) * ao
	ey = yo + np.cos(theta) * np.cos(phi) * bo
	ez = 0.0 + np.sin(theta) * 0.0
	
	fig= plt.figure()	
	ax = plt.axes(projection='3d')
	ax.contour3D(X, Y, value, 50, cmap='binary')
	ax.scatter3D(xm, ym)
	ax.plot_surface(ex, ey, ez,  rstride=4, cstride=4, color='b')

	plt.show()






