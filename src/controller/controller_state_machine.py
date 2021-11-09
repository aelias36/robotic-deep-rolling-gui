'''
STATES

disconnected - connection to EGM has timed out, trying to reconnect
manual_jog - control robot velocity through GUI buttons
toolpath - control robot based on toolpath execution
simulator - simulate the force readings so force control still works with RobotStudio
zero force - hover above the workpiece during force control commands


'''
import sys
sys.path.append('../')

import threading
import time
import numpy as np
from robot_kin import abb_6640_kinematics as kin
import general_robotics_toolbox as rox
from controller import toolpath_control
from PyQt5.QtWidgets import QFileDialog

TIMESTEP = 0.004

class ControllerStateMachine():
    def __init__(self, egm, gui):
        self.egm = egm
        self.gui = gui
        self.tp_ctrl = toolpath_control.ToolpathControl()
        self.tp_ctrl.params = {
            "force_ctrl_damping": 100.0,
            "force_epsilon": 10.0,
            "moveL_speed_lin": 1.0,
            "moveL_speed_ang": 1.0,
            "load_speed": 1.0,
            "unload_speed": 1.0
            }

        self.is_running = True
        self.state = "disconnected"
        self.gui.status_disp.setText(self.state)
        self.gui.status_disp.setStyleSheet("QLineEdit {background-color: red;}")

        work_R = np.eye(3)
        work_p = np.array([0.0, 0.0, 0.0])
        self.work_offset = rox.Transform(work_R, work_p)

        # Set up workpiece offset buttons
        self.gui.wp_os_x.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_y.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_z.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_rx.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_ry.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_rz.valueChanged.connect(self.update_wp_os)

        # Loading buttons
        self.gui.toolpath_load_button.clicked.connect(self.load_toolpath_dialog)

        self.local_robot_position = None
        self.last_q = None

    def load_toolpath_dialog(self, val):
        file_name = QFileDialog.getOpenFileName(self.gui, "Open Toolpath", filter="Toolpath Files (*.txt *.toolpath)")
        print(file_name)


    def update_wp_os(self, val):
        rx = np.deg2rad(self.gui.wp_os_rx.value())
        ry = np.deg2rad(self.gui.wp_os_ry.value())
        rz = np.deg2rad(self.gui.wp_os_rz.value())

        #self.work_offset.R = rox.rot([0,0,1], rz).dot( rox.rot([0,1,0], ry)).dot(rox.rot([1,0,0], rx))
        #print(rox.rot([0,0,1], rz))

        self.work_offset.R = rox.rpy2R([rx, ry, rz])

        self.work_offset.p[0] = self.gui.wp_os_x.value()
        self.work_offset.p[1] = self.gui.wp_os_y.value()
        self.work_offset.p[2] = self.gui.wp_os_z.value()

        print(self.work_offset)

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

        elif self.state == "toolpath":
            self.state_toolpath()

        elif self.state == "simulator":
            self.state_toolpath(simulate_force = True, zero_force = False)

        elif self.state == "zero_force":
            self.state_toolpath(simulate_force = False, zero_force = True)

        else:
            raise ValueError("Wrong state: " + str(self.state))

    def state_disconnected(self):
        res, state = self.egm.receive_from_robot(.1)
        if res:
            self.local_robot_position = None
            self.state = "manual_jog"
            self.gui.status_disp.setText(self.state)
            self.gui.status_disp.setStyleSheet("QLineEdit {}") # reset bg color
        else:
            pass

    def state_manual_jog(self):
        res, state = self.egm.receive_from_robot(.1)
        if res:
            # Clear queue
            i = 0
            while True:
                res_i, state_i = self.egm.receive_from_robot()
                if res_i: # there was another msg waiting
                    state = state_i
                    i += 1
                else: # previous msg was end of queue
                    break
            
            # if i > 0:
            #     print("Warning: Extra msgs in queue: ", i)

            t_0 = time.perf_counter()
            
            #q_meas = np.deg2rad(state.joint_angles)
            #q_meas = state.joint_angles
            
            # Calculate current position            
            # T_meas = kin.forkin(q_meas)
            # if self.local_robot_position is None:
            #     self.local_robot_position = T_meas
            #     self.last_q = q_meas

            fb = state.robot_message.feedBack.cartesian
            pos = np.array([fb.pos.x, fb.pos.y, fb.pos.z])/1000.0 # mm -> m
            quat = [fb.orient.u0, fb.orient.u1, fb.orient.u2, fb.orient.u3]
            T_meas = rox.Transform(rox.q2R(quat), pos)
            if self.local_robot_position is None:
                self.local_robot_position = T_meas

            # Display current position
            self.gui.world_x.display(T_meas.p[0])
            self.gui.world_y.display(T_meas.p[1])
            self.gui.world_z.display(T_meas.p[2])

            k, theta = rox.R2rot(T_meas.R)
            self.gui.world_r.display(np.rad2deg(theta))
            self.gui.world_rx.display(k[0])
            self.gui.world_ry.display(k[1])
            self.gui.world_rz.display(k[2])

            T_meas_work_frame = self.work_offset.inv() * T_meas

            self.gui.wp_x.display(T_meas_work_frame.p[0])
            self.gui.wp_y.display(T_meas_work_frame.p[1])
            self.gui.wp_z.display(T_meas_work_frame.p[2])

            k, theta = rox.R2rot(T_meas_work_frame.R)
            self.gui.wp_r.display(np.rad2deg(theta))
            self.gui.wp_rx.display(k[0])
            self.gui.wp_ry.display(k[1])
            self.gui.wp_rz.display(k[2])

            # Calcualate desired velocity
            jog_linear_rate = 1e-3 * self.gui.jog_linear_rate.value() # mm -> m
            jog_angular_rate = np.deg2rad(self.gui.jog_angular_rate.value())

            v_des = None
            v_rot_des = None

            # Linear
            if self.gui.plus_x.isDown():
                v_des = [1, 0.0, 0.0]
            elif self.gui.minus_x.isDown():
                v_des = [-1, 0.0, 0.0]
            elif self.gui.plus_y.isDown():
                v_des = [0.0, 1, 0.0]
            elif self.gui.minus_y.isDown():
                v_des = [0.0, -1, 0.0]
            elif self.gui.plus_z.isDown():
                v_des = [0.0, 0.0, 1]
            elif self.gui.minus_z.isDown(): 
                v_des = [0.0, 0.0, -1]
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

            if v_des is not None:
                v_des =  np.array(v_des)
            if v_rot_des is not None:
                v_rot_des = np.array(v_rot_des)

            # Rotate velocity command based on reference frame
            jog_ref_frame_num =  self.gui.jog_ref_frame_dial.value() 
            if jog_ref_frame_num == 0: # workpiece
                if v_des is not None:
                    v_des = self.work_offset.R.dot(v_des) 
                if v_rot_des is not None:
                    v_rot_des = self.work_offset.R.dot(v_rot_des)
            elif jog_ref_frame_num == 1: # world
                pass # no need to adjust - already in world frame
            elif jog_ref_frame_num == 2: # tool
                if v_des is not None:
                    v_des = self.local_robot_position.R.dot(v_des) 
                if v_rot_des is not None:
                    v_rot_des = self.local_robot_position.R.dot(v_rot_des)
            else:
                raise ValueError("Wrong value for jog reference frame dial: " + str(jog_ref_frame_num))

            # Apply velocity to local robot state
            if v_des is not None:
                self.local_robot_position.p += v_des*jog_linear_rate*TIMESTEP
            if v_rot_des is not None:
                self.local_robot_position.R = rox.rot(v_rot_des, jog_angular_rate * TIMESTEP).dot(self.local_robot_position.R)

            
            # Use local robot state to command real robot
            #q_c = kin.invkin(self.local_robot_position.R,self.local_robot_position.p, last_joints = self.last_q)[0]
            #self.last_q = q_c
            
            #self.egm.send_to_robot(q_c)

            pos = self.local_robot_position.p * 1000.0 # m -> mm
            quat = rox.R2q(self.local_robot_position.R)
            t_1 = time.perf_counter()
            send_res = self.egm.send_to_robot_cart(pos, quat)

            
            print((t_1 - t_0) / TIMESTEP * 100, '%')
        else: # EGM timed out
            self.state = "disconnected"
            self.gui.status_disp.setText(self.state)
            self.gui.status_disp.setStyleSheet("QLineEdit {background-color: red;}")

    def state_toolpath(self, simulate_force = False, zero_force = False):
        # TODO make sure toolpath is loaded
        # TODO dialog box to open toolpath
        # TODO when loading, keep track of length of toolpath
        # TODO during execution, update program counter display
        # TODO start with START button, stop with STOP button, hold with HOLD button
        #   Hold - request to hold is set with button,
        #   operation stops when force control section ends
        # TODO disable sections of GUI when running



        is_done, tool_pose = tp_ctrl.step(tool_pose, force)



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