import rpi_abb_irc5
import numpy as np
import time

egm = rpi_abb_irc5.EGM()

last_time = time.perf_counter()

try:
	while True:
		res, state = egm.receive_from_robot(.1)
		new_time = time.perf_counter()
		#print((new_time - last_time)/0.004 * 100)
		last_time = new_time
		if res:
			# Clear queue
			i = 0
			while True:
				res_i, state_i = egm.receive_from_robot()
				if res_i: # there was another msg waiting
					state = state_i
					i += 1
				else: # previous msg was end of queue
					break

			if i > 0:
				print("Number of extra msgs in queue: ", i)

			#print(state)
			#print(state.robot_message.feedBack.cartesian)
			#print()

			fb = state.robot_message.feedBack.cartesian
			pos = [fb.pos.x, fb.pos.y, fb.pos.z]
			quat = [fb.orient.u0, fb.orient.u1, fb.orient.u2, fb.orient.u3]

			pos[2] = pos[2] + 500.0
			#print(pos, quat)
			send_res = egm.send_to_robot_cart(pos, quat)
			#print("send_res:", send_res)

			#q_c = np.deg2rad(state.joint_angles)
			#q_c = state.joint_angles
			#egm.send_to_robot(q_c)
			# print(state.joint_angles)

except KeyboardInterrupt:
	raise
