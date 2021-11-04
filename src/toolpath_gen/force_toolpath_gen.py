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


Alex Elias
'''

import numpy as np
import yaml

TIMESTEP = 0.004

class ForceToolpathGen():
	def __init__(self, params):
		self.params = params

		self.x, self.y, self.z = 0.0 , 0.0, 0.0
		self.vx = 0.0
		self.force = 0.0
		self.time_ind = 0

		self.accel_slow_dist = None

		self.accelerated_to_fast = False
		self.accel_fast_dist = None


	def gen_single_roll(self):
		# return True -> go to next mode
		# return False -> repeat same mode
		self.x = self.params['margin_length']
		mode = "load"

		x_hist = []
		force_hist = []

		print("starting load")
		while True:
			# Write down current state
			x_hist.append(self.x)
			force_hist.append(self.force)
			# Evolve state
			if mode == "load":
				if self.load():
					mode = "ramp_up"
					print("starting ramp_up")
					print("vel:", self.vx)
					print("pos:", self.x)
					print("force:", self.force)

			elif mode == "ramp_up":
				if self.ramp_up():
					mode = "constant_force"
					print("starting constant_force")
					print("accel_slow_dist:", self.accel_slow_dist)
					print("vel:", self.vx)
					print("pos:", self.x)
					print("force:", self.force)

			elif mode == "constant_force":
				if self.constant_force():
					mode = "ramp_down"
					print("starting ramp_down")
			elif mode == "ramp_down":
				if self.ramp_down():
					mode = "unload"
					print("starting unload")
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


def main():
	with open('toolpath_constants.yaml', 'r') as file:
		params = yaml.safe_load(file)

	params['margin_length'] = 1e-3
	params['stepover'] = .01e-3
	params['f_max'] = 1000

	print(params)

	gen = ForceToolpathGen(params)
	x_hist, force_hist = gen.gen_single_roll()
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


if __name__ == '__main__':
	main()