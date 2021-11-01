import rpi_abb_irc5
import numpy as np

egm = rpi_abb_irc5.EGM()

try:
	while True:
		res, state = egm.receive_from_robot(.1)
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

			q_c = np.deg2rad(state.joint_angles)
			egm.send_to_robot(q_c)
			#print(q_c)
		#print(res, state)

except KeyboardInterrupt:
	raise
