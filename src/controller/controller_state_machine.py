'''
STATES

disconnected - connection to EGM has timed out, trying to reconnect
manual_jog - control robot velocity through GUI buttons



'''
import sys
sys.path.append('../')

import threading
import time
import numpy as np
from robot_kin import abb_6640_kinematics as kin
import general_robotics_toolbox as rox

JOG_LINEAR_VEL = 0.1 # m/s
JOG_ANGULAR_VEL = np.deg2rad(30) # rad/sec
TIMESTEP = 0.004

class ControllerStateMachine():
    def __init__(self, egm, gui):
        self.egm = egm
        self.gui = gui

        self.is_running = True
        self.state = "disconnected"
        self.gui.status_disp.setText(self.state)
        self.gui.status_disp.setStyleSheet("QLineEdit {background-color: red;}")

        self.local_robot_position = None
        self.last_q = None

    def start_loop(self):
        self.thread = threading.Thread(target = self.state_machine_loop)
        self.thread.start()

    def stop_loop(self):
        self.is_running = False
        self.thread.join()

    def state_machine_loop(self):
        while self.is_running:
            self.step()


    def step(self):
        if self.state == "disconnected":
            self.state_disconnected()

        elif self.state == "manual_jog":
            self.state_manual_jog()

        else:
            raise ValueError("Wrong state: " + str(self.state))


    def state_disconnected(self):
        res, state = self.egm.receive_from_robot(.1)
        if res:
            self.local_robot_position = None
            for i in range(100):
                    self.egm.receive_from_robot() # clear buffer
            self.state = "manual_jog"
            self.gui.status_disp.setText(self.state)
            self.gui.status_disp.setStyleSheet("") # reset bg color
        else:
            pass

    def state_manual_jog(self):
        res, state = self.egm.receive_from_robot(.1)
        if res:
            q_meas = np.deg2rad(state.joint_angles)
            
            # Calculate current position            
            T_meas = kin.forkin(q_meas)
            if self.local_robot_position is None:
                self.local_robot_position = T_meas
                self.last_q = q_meas

            # Display current position
            self.gui.world_x.display(T_meas.p[0])
            self.gui.world_y.display(T_meas.p[1])
            self.gui.world_z.display(T_meas.p[2])

            k, theta = rox.R2rot(T_meas.R)
            self.gui.world_r.display(np.rad2deg(theta))
            self.gui.world_rx.display(k[0])
            self.gui.world_ry.display(k[1])
            self.gui.world_rz.display(k[2])

            # Calcualate desired velocity
            v_des = None
            v_rot_des = None

            # Linear
            if self.gui.plus_x.isDown():
                v_des = [JOG_LINEAR_VEL, 0.0, 0.0]
            elif self.gui.minus_x.isDown():
                v_des = [-JOG_LINEAR_VEL, 0.0, 0.0]
            elif self.gui.plus_y.isDown():
                v_des = [0.0, JOG_LINEAR_VEL, 0.0]
            elif self.gui.minus_y.isDown():
                v_des = [0.0, -JOG_LINEAR_VEL, 0.0]
            elif self.gui.plus_z.isDown():
                v_des = [0.0, 0.0, JOG_LINEAR_VEL]
            elif self.gui.minus_z.isDown(): 
                v_des = [0.0, 0.0, -JOG_LINEAR_VEL]
            # Angular
            elif self.gui.plus_rx.isDown():
                v_rot_des = [1.0, 0.0, 0.0]
            elif self.gui.minus_rx.isDown():
                v_rot_des = [-1.0, 0.0, 0.0]
            elif self.gui.plus_ry.isDown():
                v_rot_des = [0.0, 1.0, 0.0]
            elif self.gui.minus_ry.isDown():
                v_rot_des = [0.0, -1.0, 0.0]
            elif self.gui.plus_rz.isDown():
                v_rot_des = [0.0, 0.0, 1.0]
            elif self.gui.minus_rz.isDown(): 
                v_rot_des = [0.0, 0.0, -1.0]

            # Apply velocity to local robot state
            if v_des is not None:
                self.local_robot_position.p += np.array(v_des)*TIMESTEP
            if v_rot_des is not None:
                self.local_robot_position.R = rox.rot(np.array(v_rot_des), JOG_ANGULAR_VEL * TIMESTEP).dot(self.local_robot_position.R)

            # Use local robot state to command real robot
            q_c = kin.invkin(self.local_robot_position.R,self.local_robot_position.p, last_joints = self.last_q)[0]
            
            self.last_q = q_c
            self.egm.send_to_robot(q_c)
        
        else: # EGM timed out
            self.state = "disconnected"
            self.gui.status_disp.setText(self.state)
            self.gui.status_disp.setStyleSheet("QLineEdit {background-color: red;}")



def main():
    import sys
    sys.path.append('../')
    from rpi_abb_irc5 import rpi_abb_irc5
    from gui import rolling_gui
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer

    egm = rpi_abb_irc5.EGM()
    
    app = QApplication(sys.argv)
    gui = rolling_gui.Window()
    gui.show()

    state_machine = ControllerStateMachine(egm, gui)
    state_machine.start_loop()

    
    import signal
    signal.signal(signal.SIGINT, lambda arg1, arg2 : app.quit())
    timer = QTimer()
    timer.start(50) # To wait for ctrl-C signal
    timer.timeout.connect(lambda: None)

    try:
        app.exec()
    finally:
        state_machine.stop_loop()

if __name__ == '__main__':
    main()