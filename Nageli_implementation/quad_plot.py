import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Ellipse, Rectangle
from numpy import*

def visibility_score(x, y, xm, ym, xo, yo, beta):
	proj = (xm - x) * (xo - x) + (ym - y) * (yo - y)
	rh = (xm - x) * (xm - x) + (ym - y) * (ym - y)
	
	rt = (xo - x) * (xo - x) + (yo - y) * (yo - y) 		
	dv = proj / rh
	temp = proj - rt + 1
	score =  dv * (dv / (abs(dv) + 0.000000000001) + 1) * (temp / (abs(temp) + 0.000000000001) + 1)
	
	return score

def get_values():
	x = []
	y = []
	obsx0 = []
	obsy0 = []
	obsx1 = []
	obsy1 = []
	obsx2 = []
	obsy2 = []
	x_c = []
	y_c = []
	r_c = []


	f = open("x.txt", "r")
	f1 = f.readlines()
	for data in f1:
		x.append(float(data))
	
	f = open("y.txt", "r")
	f1 = f.readlines()
	for data in f1:
		y.append(float(data))

	f = open("obsx0.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsx0.append(float(data))

	f = open("obsy0.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsy0.append(float(data))

	f = open("obsx1.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsx1.append(float(data))

	f = open("obsy1.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsy1.append(float(data))

	f = open("obsx2.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsx2.append(float(data))

	f = open("obsy2.txt", "r")
	f1 = f.readlines()
	for data in f1:
		obsy2.append(float(data))
	
	"""f = open("x_c.txt", "r")
	f1 = f.readlines()
	for data in f1:
		x_c.append(float(data))

	f = open("y_c.txt", "r")
	f1 = f.readlines()
	for data in f1:
		y_c.append(float(data))

	f = open("r_c.txt", "r")
	f1 = f.readlines()
	for data in f1:
		r_c.append(float(data))"""

	return x, y, obsx0, obsy0, obsx1, obsy1, obsx2, obsy2

if __name__ == "__main__":
	
	xm = -1.0
	ym = 5.0
	
	ao = 0.35+0.25

	x, y, obsx0, obsy0, obsx1, obsy1, obsx2, obsy2 = get_values()

	xx = linspace(-8, 8, 100)
	yy = linspace(-8, 8, 100)
	
	X, Y = meshgrid(xx, yy)
	

	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)
	
	xlim = 4
	ylim = 3
	while(1):
		for i in range(0,len(x)):
			
			plt.clf()
			# plt.plot([-xlim, xlim, xlim, -xlim, -xlim], [ylim, ylim, -ylim, -ylim, ylim ])
			plt.plot([xm, x[i]], [ym, y[i]])
			# circle0 = plt.Circle((obsx0[i], obsy0[i]), ao, color='black', fill=True)
			# circle1 = plt.Circle((obsx1[i], obsy1[i]), ao, color='black', fill=True)
			# circle2 = plt.Circle((xm, ym), ao, color='black', fill=False, linestyle = '-')
			# circle3 = plt.Circle((obsx2[i], obsy2[i]), ao, color='black', fill=True)
			
			# circle0 = plt.Circle((1.2, 3.0), ao, color='black', fill=True)
			# circle1 = plt.Circle((-3.0, 1.0), ao, color='black', fill=True)
			circle0 = Ellipse(xy=[1.2, 3.0], width=1.6*2, height=0.6*2, angle=0)
			circle1 = Ellipse(xy=[-3.0, 1.0], width=0.7*2, height=0.7*2, angle=0)

			circle2 = plt.Circle((xm, ym), ao, color='black', fill=False, linestyle = '-')
			
			# score0 = visibility_score(x[i], y[i], xm, ym, obsx0[i], obsy0[i], 0)
			# score1 = visibility_score(x[i], y[i], xm, ym, obsx1[i], obsy1[i], 0)
			# score2 = visibility_score(x[i], y[i], xm, ym, obsx2[i], obsy2[i], 0)
			# plt.text(-5, -5, 'Score0 = %s\nScore1 = %s\nScore2 = %s' % (round(score0, 2), round(score1, 2), round(score2, 2)))
			"""fov0 = plt.Circle((x_c[i*10], y_c[i*10]), r_c[i*10], color='r', fill=False,  linewidth = 1.0)
			fov1 = plt.Circle((x_c[i*10+1], y_c[i*10+1]), r_c[i*10+1], color='r', fill=False, linewidth = 1.0)
			fov2 = plt.Circle((x_c[i*10+2], y_c[i*10+2]), r_c[i*10+2], color='r', fill=False, linewidth = 1.0)
			fov3 = plt.Circle((x_c[i*10+3], y_c[i*10+3]), r_c[i*10+3], color='r', fill=False, linewidth = 1.0)
			fov4 = plt.Circle((x_c[i*10+4], y_c[i*10+4]), r_c[i*10+4], color='r', fill=False, linewidth = 1.0)
			fov5 = plt.Circle((x_c[i*10+5], y_c[i*10+5]), r_c[i*10+5], color='r', fill=False, linewidth = 1.0)
			fov6 = plt.Circle((x_c[i*10+6], y_c[i*10+6]), r_c[i*10+6], color='r', fill=False, linewidth = 1.0)
			fov7 = plt.Circle((x_c[i*10+7], y_c[i*10+7]), r_c[i*10+7], color='r', fill=False, linewidth = 1.0)
			fov8 = plt.Circle((x_c[i*10+8], y_c[i*10+8]), r_c[i*10+8], color='r', fill=False, linewidth = 1.0)
			fov9 = plt.Circle((x_c[i*10+9], y_c[i*10+9]), r_c[i*10+9], color='r', fill=False, linewidth = 1.0)"""
			

			ax = fig.add_subplot(1, 1, 1)
			ax.add_artist(circle0)
			ax.add_artist(circle1)
			ax.add_artist(circle2)
			# ax.add_artist(circle3)			
	
			ax.scatter(x[i], y[i], linewidth=0.0)			

			ax.plot(x, y)
			ax.scatter(xm, ym)

			ax.set_xlabel('x')
			ax.set_ylabel('y')
			
			ax.set_xlim(-10, 10)
			ax.set_ylim(-10, 10)
			
			plt.draw()
			plt.pause(0.000001)
		break	
	plt.show()




















