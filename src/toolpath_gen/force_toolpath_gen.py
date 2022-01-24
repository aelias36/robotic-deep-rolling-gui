'''
Make a deep rolling toolpath

Load:
	Without moving across the part, ramp force from (near) 0 to some minimum value
Ramp up:
	Tool starts moving slowly across part, and force ramps up linearly w.r.t distance
Constant force:
	Force stays constant, and tools speeds up to max speed, then decelerates to slow speed
Ramp down:
	While tool is moving slowly again, force decreases linearly w.r.t. time
Unload:
	Without moving across the part, decrease force to (near) 0



Workpiece coordinate axes:
	X points in the length direction - rolling direction
	Y points in the width direction - stepover direction
	Z points in the surface normal direction (away from part)
	(0,0,0) is on the corner of the bounding box
	Positive X and Y axes touch the bounding box of the part



PARAMETERS

Geometry:
	part_length: Bounding box distance in X (rolling) direction
	part_width: Bounding box distance in Y (stepover) direction
	margin_width: Width of unrolled area on each side (X  / rolling direction)
	margin_length: Length of unrolled area on each side (Y / stepover direction)
	stepover: Width between centers of adjacent rolls (in Y direction)
	hover_height: Height of tool during stepover

Force Profile:
	f_load_unload_rate: Rate w.r.t. to ramp force during loading and unloading
	f_max: Maximum force, used during constant force section
	f_min_prop: Proportion of F_max applied at very beginning and end of roll
	f_ramp_dist: Distance to ramp from F_min to F_max, and F_max to F_min  

Velocity Profile:
	accel: Acceleration used to ramp between different velocities
	v_slow: Slower velocity used at beginning and end of roll
	v_fast: Faster velocity used in middle of roll
	v_slow_dist: Distance at beginning and end of roll to travel at <= v_slow
	v_stepover: Velocity used during a stepover


Alex Elias
'''

import numpy as np
import yaml
import datetime
import os

TIMESTEP = 0.004

class GenFullToolpath():
	def __init__(self, params):
		self.params = params
		self.single_gen = SingleRollGen(params)

	def header_txt(self):
		txt = "# Deep rolling toolpath\n"
		txt += "# Generated by {}\n".format(os.path.basename(__file__))
		txt += "# Generated on {}\n".format(datetime.datetime.now())
		txt += "#\n# --- Parameters ---\n"
		for key, val in self.params.items():
			txt += "# {}: {}\n".format(key, val)
		txt += "#\n#\n"
		return txt

	def toolpath(self):
		assert(self.params['part_width'] > 2.0 * self.params['margin_width'])

		# Zig - roll in positive X direction
		# Zag - roll in negative X direction
		zig_x, zig_f = self.single_gen.gen_single_roll()
		zig_x = np.array(zig_x)
		zig_f = np.array(zig_f)

		zag_x = self.params['part_length'] - zig_x
		zag_f = np.copy(zig_f)

		roll_y_positions = np.arange(
			self.params['margin_width'],
			self.params['part_width'] - self.params['margin_width'],
			self.params['stepover'],
			dtype=float)

		toolpath = ""

		# Move to hover position above first roll
		toolpath += "moveL {} {} {} 1 0 0 0\n".format(self.params['margin_length'], self.params['margin_width'], self.params['hover_height'])
		toolpath += "tare\n"

		y_index = 0
		while True:
			# Loading
			toolpath += "loadZ\n"

			# Zig
			for i in range(len(zig_x)):
				toolpath += "forceCtrlZ {} {} {} 1 0 0 0\n".format(zig_x[i], roll_y_positions[y_index], zig_f[i])

			# Unloading
			toolpath += "unloadZ {}\n".format(self.params['hover_height'])

			# Check if done
			if y_index == len(roll_y_positions) - 1:
				break

			# Stepover
			y_start = roll_y_positions[y_index]
			y_index += 1
			y_end = roll_y_positions[y_index]

			y = y_start
			while(y + self.params['v_stepover'] * TIMESTEP < y_end):
				y += self.params['v_stepover'] * TIMESTEP;
				toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(
					self.params['part_length'] - self.params['margin_length'], y, self.params['hover_height'])
			toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(
				self.params['part_length'] - self.params['margin_length'], y_end, self.params['hover_height'])

			# Loading
			toolpath += "loadZ\n"

			# Zag
			for i in range(len(zag_x)):
				toolpath += "forceCtrlZ {} {} {} 1 0 0 0\n".format(zag_x[i], roll_y_positions[y_index], zag_f[i])

			# Unloading
			toolpath += "unloadZ {}\n".format(self.params['hover_height'])

			# Check if done
			if y_index == len(roll_y_positions) - 1:
				break

			# Stepover
			y_start = roll_y_positions[y_index]
			y_index += 1
			y_end = roll_y_positions[y_index]

			y = y_start
			while(y + self.params['v_stepover'] * TIMESTEP < y_end):
				y += self.params['v_stepover'] * TIMESTEP;
				toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(
					self.params['margin_length'], y, self.params['hover_height'])
			toolpath += "posCtrl {} {} {} 1 0 0 0\n".format(
				self.params['margin_length'], y_end, self.params['hover_height'])

		toolpath += "# End of toolpath"

		stats = "# --- Statistics ---\n"
		line_count = toolpath.count('\n')
		stats += "# Number of command lines: {}\n".format(line_count)
		stats += "# Time to run not including loading / unloading: {}\n".format(datetime.timedelta(seconds=line_count * 0.004))
		stats += "# Number of rolls: {}\n".format(len(roll_y_positions))
		stats += "#\n#\n"

		return self.header_txt() + stats + toolpath


class SingleRollGen():
	'''
	Generate the X and force profile
	A full path has a bunch of these single rolls
	'''
	def __init__(self, params):
		self.params = params

		self.x = 0.0
		self.vx = 0.0
		self.force = 0.0

		self.accel_slow_dist = None

		self.accelerated_to_fast = False
		self.accel_fast_dist = None


	def gen_single_roll(self):
		
		self.x = self.params['margin_length']
		mode = "load"

		x_hist = []
		force_hist = []

		
		# return True -> go to next mode
		# return False -> repeat same mode
		while True:
			# Write down current state
			x_hist.append(self.x)
			force_hist.append(self.force)
			# Evolve state
			if mode == "load":
				if self.load():
					mode = "ramp_up"
					# print("starting ramp_up")
					# print("vel:", self.vx)
					# print("pos:", self.x)
					# print("force:", self.force)

			elif mode == "ramp_up":
				if self.ramp_up():
					mode = "constant_force"
					# print("starting constant_force")
					# print("accel_slow_dist:", self.accel_slow_dist)
					# print("vel:", self.vx)
					# print("pos:", self.x)
					# print("force:", self.force)

			elif mode == "constant_force":
				if self.constant_force():
					mode = "ramp_down"
					#print("starting ramp_down")

			elif mode == "ramp_down":
				if self.ramp_down():
					mode = "unload"
					#print("starting unload")

			elif mode == "unload":
				if self.unload():
					break
		
		x_hist.append(self.x)
		force_hist.append(self.force)

		return x_hist, force_hist

	def load(self):
		self.force += self.params['f_load_unload_rate'] * TIMESTEP
		
		if self.force >= self.params['f_min_prop'] * self.params['f_max']:
			self.force = self.params['f_min_prop'] * self.params['f_max']
			return True
		
		return False

	def ramp_up(self):
		# Increase velocity as needed
		if self.vx < self.params['v_slow']:
			self.vx += self.params['accel'] * TIMESTEP
			if self.vx >= self.params['v_slow']:
				self.vx = self.params['v_slow']
				if self.accel_slow_dist is None:
					self.accel_slow_dist = self.x

		# Change position based on velocity
		self.x += self.vx * TIMESTEP

		# Force is based on distance
		dist_from_edge = self.x - self.params['margin_length']
		if dist_from_edge < self.params['f_ramp_dist']:
			prop = dist_from_edge / self.params['f_ramp_dist'] 
			self.force = prop * self.params['f_max'] + (1-prop) * self.params['f_min_prop'] * self.params['f_max']
		else:
			self.force = self.params['f_max']
			return True

		return False

	def constant_force(self):
		# Start increasing velocity after v_slow_dist
		if not self.accelerated_to_fast and self.x >= self.params['margin_length'] + self.params['v_slow_dist']:
			self.vx += self.params['accel'] * TIMESTEP
			if self.vx >= self.params['v_fast']:
				self.vx = self.params['v_fast']
				self.accelerated_to_fast = True
				self.accel_fast_dist = self.x

		# Decrease velocity once we reach a symmetric point
		if self.accelerated_to_fast and self.x >= self.params['part_length'] - self.accel_fast_dist:
			self.vx -= self.params['accel'] * TIMESTEP
			if self.vx < self.params['v_slow']:
				self.vx = self.params['v_slow']

		# Change position based on velocity
		self.x += self.vx * TIMESTEP

		if self.x >= self.params['part_length'] - self.params['margin_length'] - self.params['f_ramp_dist']:
			return True

		return False

	def ramp_down(self):
		# Decrease velocity once we reach a symmetric point
		if self.x >= self.params['part_length'] - self.accel_slow_dist:
			self.vx -= self.params['accel'] * TIMESTEP
			if self.vx < 0:
				self.vx = 0
				
		# change position based on velocity
		self.x += self.vx * TIMESTEP

		# Force based on distance
		dist_from_edge = self.params['part_length'] - self.params['margin_length'] - self.x
		prop = dist_from_edge / self.params['f_ramp_dist'] 
		self.force = prop * self.params['f_max'] + (1-prop) * self.params['f_min_prop'] * self.params['f_max']

		return self.vx == 0


	def unload(self):
		self.force -= self.params['f_load_unload_rate'] * TIMESTEP
		
		if self.force < 0:
			self.force = 0
			return True
		
		return False


def test_single_gen():
	with open('toolpath_constants.yaml', 'r') as file:
		params = yaml.safe_load(file)

	params['margin_length'] = 1e-3
	params['stepover'] = .01e-3
	params['f_max'] = 1000

	print(params)

	single_gen = SingleRollGen(params)
	x_hist, force_hist = single_gen.gen_single_roll()
	#print(x_hist)
	#print(force_hist)

	print(params['part_length'] - params['margin_length'])
	print(x_hist[-1])
	print(params['part_length'] - params['margin_length'] - x_hist[-1])

	import matplotlib.pyplot as plt
	plt.plot(np.diff(x_hist) / TIMESTEP)
	plt.ylabel('Speed (m/s)')
	plt.xlabel('Time (sample #)')

	plt.figure()
	plt.plot(x_hist)
	plt.ylabel('Position (m)')
	plt.xlabel('Time (sample #)')

	plt.figure()
	plt.plot(force_hist)
	plt.ylabel('Force (N)')
	plt.xlabel('Time (sample #)')

	plt.figure()
	plt.plot(x_hist, force_hist)
	plt.ylabel('Force (N)')
	plt.xlabel('Position (m)')


	plt.show()

def main():
	with open('toolpath_constants.yaml', 'r') as file:
		params = yaml.safe_load(file)

	params['margin_length'] = 0.15 * 25.4e-3
	params['stepover'] = 0.006 * 25.4e-3
	params['f_max'] = 1779 # 400 lb
	params['f_ramp_dist'] = 0.15 * 25.4e-3
	params['v_slow_dist'] = 0.25 * 25.4e-3

	gen = GenFullToolpath(params)

	#print(gen.header_txt())
	toolpath = gen.toolpath()
	with open('RTX_v3.toolpath', 'w') as file:
		file.write(toolpath)


if __name__ == '__main__':
	main()
