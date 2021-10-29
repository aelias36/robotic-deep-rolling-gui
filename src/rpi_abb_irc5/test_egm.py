import rpi_abb_irc5
import numpy as np

egm = rpi_abb_irc5.EGM()

try:
	while True:
		res, state = egm.receive_from_robot(.1)
		if res:
			q_c = np.deg2rad(state.joint_angles)
			egm.send_to_robot(q_c)
		print(res, state)
except KeyboardInterrupt:
	raise
