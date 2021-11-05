'''
toolpath_control.py
Alex Elias

Calculates the desired tool pose using a toolpath text file,
along with force feedback and the previous desired tool pose

PARAMETERS

force_ctrl_damping: damping for force control
	Higher damping means less velocity for the same force
force_epsilon: smallest value of force needed be considered touching
	Make high enough 
moveL_speed_lin: linear tool speed during moveL commands
moveL_speed_ang: angular tool speed during moveL commands
'''


from collections import namedtuple
import general_robotics_toolbox as rox

import numpy as np

TIMESTEP = 0.004

class ToolpathControl():
	def __init__(self):
		self.commands = []
		self.program_counter = 0


		self.CmdMoveL = namedtuple("CmdMoveL", "x y z q0 qx qy qz")
		self.CmdLoadZ = namedtuple("CmdLoadZ", "")
		self.CmdForceCtrlZ = namedtuple("CmdForceCtrlZ", "x y fz q0 qx qy qz")
		self.CmdPosCtrl = namedtuple("CmdPosCtrl", "x y z q0 qx qy qz")
		self.CmdUnloadZ = namedtuple("CmdUnloadZ", "")

	def load_toolpath(self, toolpath_lines):
		self.commands = []
		self.program_counter = 0

		for line in toolpath_lines:
			line_sep = line.split()
			if line[0] == "#": # Comment
				continue
			elif line_sep[0] == "moveL":
				self.commands.append(
					self.CmdMoveL(*[float(x) for x in line_sep[1:]]))

			elif line_sep[0] == "LoadZ":
				self.commands.append(
					self.CmdLoadZ())

			elif line_sep[0] == "forceCtrlZ":
				self.commands.append(
					self.CmdForceCtrlZ(*[float(x) for x in line_sep[1:]]))

			elif line_sep[0] == "posCtrl":
				self.commands.append(
					self.CmdPosCtrl(*[float(x) for x in line_sep[1:]]))

			elif line_sep[0] == "unloadZ":
				self.commands.append(
					self.CmdUnloadZ())

	def step(self, tool_pose, force):
		# TODO check if a program is loaded

		current_cmd = self.commands[self.program_counter]

		# return True -> move to next cmd
		# return False -> stay on command
		if type(current_cmd) is self.CmdMoveL:
			inc, T = self.moveL(current_cmd, tool_pose, force)

		elif type(current_cmd) is self.CmdLoadZ:
			inc, T = self.loadZ(tool_pose, force)

		elif type(current_cmd) is self.CmdForceCtrlZ:
			inc, T = self.forceCtrlZ(cmd, tool_pose, force)

		elif type(current_cmd) is self.CmdPosCtrl:
			inc, T = self.posCtrl(cmd)

		elif type(current_cmd) is self.CmdUnloadZ:
			inc, T = self.unloadZ(cmd, tool_pose, force)
		
		else:
			raise ValueError('Wrong command type: ' + str(current_cmd))

		# TODO check if we're at the end of the program
		if inc:
			self.program_counter += 1

		return T

	def moveL(self, cmd, tool_pose, force):
		# Calculate linear and angular errors

		# Small error -> snap to correct position
		# Big error -> reduce error with constant vel

		pass 

	def loadZ(self, tool_pose, force):
		# TODO make sure to tare F/T sensor
		f_z_meas = force[5]
		pass

	def forceCtrlZ(self, cmd, tool_pose, force):
		# TODO limit to force_epsilon so we don't lose contact

		# TODO lookahead
		f_z_meas = force[5]
		R = rox.q2R(cmd.q0, cmd.qx, cmd.qy, cmd.qz)

		# TODO make sure this sign makes sense
		v_z = ( f_z_meas - cmd.fz ) / self.params['force_ctrl_damping']
		z = cmd.z + v_z * TIMESTEP

		p = [cmd.x, cmd.y, z]
		return true, rox.Transform(R,p)


	def posCtrl(self, cmd):
		R = rox.q2R(cmd.q0, cmd.qx, cmd.qy, cmd.qz)
		p = [cmd.x, cmd.y, cmd.z]
		
		return true, rox.Transform(R,p)

	def unloadZ(self, tool_pose, force):
		# TODO need to include a Z height to move to
		f_z_meas = force[5]
		pass

def main():
	file = "../toolpath_gen/test_force_toolpath.txt"
	with open(file) as f:
		lines = f.readlines()

	tp_ctrl = ToolpathControl()
	tp_ctrl.load_toolpath(lines)

	for i in range(20):
		print(tp_ctrl.commands[i])



if __name__ == '__main__':
	main()