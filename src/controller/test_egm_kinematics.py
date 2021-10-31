import sys
sys.path.append('../')


from robot_kin import abb_6640_kinematics as kin
from rpi_abb_irc5 import rpi_abb_irc5
import numpy as np

egm = rpi_abb_irc5.EGM()

buffer_cleared = False

try:
    while True:
        res, state = egm.receive_from_robot(.1)
        if res:
            if not buffer_cleared:
                for i in range(100):
                    egm.receive_from_robot()
                buffer_cleared = True

            q = np.deg2rad(state.joint_angles)

            T = kin.forkin(q)
            #print("Robot T:")
            #print(T)

            T.p[2] += 1e-2
            # The actual robot speed depends on the delay
            # between sending a command and the robot executing it
            # Only doing this for testing

            q_c = kin.invkin(T.R,T.p, last_joints = q)[0]
            #print("Inv q:")
            #print(q_c)

            egm.send_to_robot(q_c.tolist())
        #print(res, state)
        else:
            buffer_cleared = False
except KeyboardInterrupt:
    raise
