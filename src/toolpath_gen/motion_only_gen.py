'''
Make a deep rolling toolpath that only uses motion control - no force control

Used for testing

Alex Elias
'''

import numpy as np

TIMESTEP = 0.004

def motion_only_gen(part_width, part_length, margin_width, stepover, vel_ramp_dist, max_vel, hover_dist, stepover_speed):
	# X points in the length direction
	# Y points in the width direction
	# Z points in the surface normal direction (away from part)
	assert(part_width > 2.0 * margin_width)
	
	toolpath = ""

	roll_x_positions = trapezoid(max_vel, vel_ramp_dist, part_length)
	print("roll_x_positions: ", roll_x_positions)
	roll_y_positions = np.arange(margin_width, part_width - margin_width + stepover, stepover, dtype=float)
	print("roll_y_positions: ", roll_y_positions)

	# Move to hover position above first roll

	toolpath += "moveL {} {} {} 1 0 0 0\n".format(0, margin_width, hover_dist)

	y_index = 0
	while True:
		# Skip loading
		
		# Zig
		for x_roll in roll_x_positions:
			toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(x_roll, roll_y_positions[y_index], hover_dist)
		
		# Skip unloading
		
		# Check if done
		if y_index == len(roll_y_positions) - 1:
			break

		# Stepover
		y_start = roll_y_positions[y_index]
		y_index += 1
		y_end = roll_y_positions[y_index]

		y = y_start
		while(y + stepover_speed * TIMESTEP < y_end):
			y += stepover_speed * TIMESTEP;
			toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(part_length, y, hover_dist)
		toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(part_length, y_end, hover_dist)

		# Skip loading

		# Zag
		for x_roll in roll_x_positions:
			toolpath += "posCtrl  {} {} {} 1 0 0 0\n".format(part_length -  x_roll, y, hover_dist)
		# Skip unloading

		# Chek if done
		if y_index == len(roll_y_positions) - 1:
			break

		# Stepover
		y_start = roll_y_positions[y_index]
		y_index += 1
		y_end = roll_y_positions[y_index]

		y = y_start
		while(y + stepover_speed * TIMESTEP < y_end):
			y += stepover_speed * TIMESTEP;
			toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(0, y, hover_dist)
		toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(0, y_end, hover_dist)
	return toolpath



def trapezoid(max_vel, vel_ramp_dist, dist):
	assert(dist > 2.0 * vel_ramp_dist)

	accel = max_vel**2.0 / (2.0*vel_ramp_dist)

	vel_01 = np.arange(0.0, max_vel, accel * TIMESTEP, dtype=float)
	pos_01 = np.cumsum(vel_01 * TIMESTEP)


	vel_23 = np.arange(max_vel, 0.0, -accel* TIMESTEP, dtype=float)
	pos_23 = np.cumsum(vel_23 * TIMESTEP)
	pos_23 += dist - pos_23[-1]

	pos_12 = np.arange(pos_01[-1], pos_23[0], max_vel*TIMESTEP, dtype=float)

	pos = np.concatenate([pos_01, pos_12, pos_23])
	return pos
	#return pos_01

def test_trapezoid():
	import matplotlib.pyplot as plt
	max_vel = 1.0e-2
	vel_ramp_dist = 40e-2
	dist = 100e-2

	pos = trapezoid(max_vel, vel_ramp_dist, dist)

	figure, axis = plt.subplots(2, 1)
	axis[0].plot(pos)
	axis[1].plot(np.diff(pos))
	plt.show()

def main():
	part_width   = 2.54e-2 # m ( 1 inch )
	part_length = 3 * 2.54e-2 # m
	margin_width = 2e-3 # m


	stepover = 0.15e-3 # m
	vel_ramp_dist = 2e-2 # m
	max_vel = 0.05 # m/s

	hover_dist = 5e-3 # m

	stepover_speed = 0.005 # m/s

	toolpath = motion_only_gen(part_width, part_length, margin_width, stepover, vel_ramp_dist, max_vel, hover_dist, stepover_speed)
	with open('test_toolpath.txt', 'w') as file:
		file.write(toolpath)

if __name__ == '__main__':
	main()