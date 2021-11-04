import matplotlib.pyplot as plt
import numpy as np


def main():
	file = "test_force_toolpath.txt"
	with open(file) as f:
		lines = f.readlines()

	x = []
	y = []
	z = []
	f = []

	for line in lines:
		line_sep = line.split()
		if line_sep[0] == "forceCtrlZ":
			x.append(float(line_sep[1]))
			y.append(float(line_sep[2]))
			z.append(z[-1])
			f.append(float(line_sep[3]))
		elif line_sep[0] == "posCtrl" or line_sep[0] == "moveL":
			x.append(float(line_sep[1]))
			y.append(float(line_sep[2]))
			z.append(float(line_sep[3]))
			f.append(0.0)

	#ax = plt.axes(projection='3d')	
	#ax.plot3D(np.array(x), np.array(y), np.array(z))

	# plt.locator_params(axis='y', nbins=6)
	# plt.locator_params(axis='x', nbins=10)
	# figure, axis = plt.subplots(3, 1)
	# axis[0].plot(np.array(x))
	# axis[1].plot(np.array(y))
	# axis[2].plot(np.array(z))
	#set_axes_equal(ax)
	
	plt.plot(x, y)
	plt.title('XY')
	plt.figure()
	plt.plot(x)
	plt.title('X')
	plt.figure()
	plt.plot(y)
	plt.title('Y')
	plt.figure()
	plt.plot(f)
	plt.title('Force')
	plt.show()
	

# https://newbedev.com/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to-x-and-y
def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

if __name__ == '__main__':
	main()