'''
MODES

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
import yaml
from collections import namedtuple
import dataclasses

from robot_kin import abb_6640_kinematics as kin
import general_robotics_toolbox as rox
from controller import toolpath_control
from ft_simulation import ft_simulation

from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer

TIMESTEP = 0.004

@dataclasses.dataclass
class SafetyStatus:
    # EGM
    egm_connected: bool = False
    rapid_running: bool = False
    motors_on: bool = False
    # F/T sensor
    ft_connected: bool = False
    ft_status_normal: bool = False
    # Toolpath execution
    toolpath_loaded: bool = False
    toolpath_config_loaded: bool = False
    wp_os_loaded: bool = False
    tool_os_loaded: bool = False
    ft_os_loaded: bool = False
    f_below_pos_thresh: bool = False
    f_below_f_ctrl_thresh: bool = False

    def egm_ok(self):
        return self.egm_connected and self.rapid_running and self.motors_on

    def ft_ok(self):
        return self.ft_connected and self.ft_status_normal

    def toolpath_ready(self):
        return self.egm_ok() \
            and self.toolpath_loaded \
            and self.toolpath_config_loaded \
            and self.wp_os_loaded \
            and self.tool_os_loaded \
            and self.ft_os_loaded # TODO add FT requirements


class ControllerStateMachine(QtCore.QObject):
    # https://medium.com/@armin.samii/avoiding-random-crashes-when-multithreading-qt-f740dc16059
    # Signals and slots must be used to ensure thread safety
    # Any GUI access needs to be done on the main thread
    signal_display_positions = QtCore.pyqtSignal(rox.Transform, rox.Transform)
    signal_display_tool_offset = QtCore.pyqtSignal(rox.Transform)
    signal_display_ft_offset = QtCore.pyqtSignal(rox.Transform)
    signal_display_wp_offset = QtCore.pyqtSignal(rox.Transform)
    signal_mode = QtCore.pyqtSignal(str)
    signal_status = QtCore.pyqtSignal(str, str)
    signal_gui_lockout_section_enable = QtCore.pyqtSignal(bool)
    signal_current_cmd_number = QtCore.pyqtSignal(str)
    signal_progress_bar = QtCore.pyqtSignal(int)
    signal_current_cmd_text = QtCore.pyqtSignal(str)
    signal_toolpath_name = QtCore.pyqtSignal(str)
    signal_status_lights = QtCore.pyqtSignal(SafetyStatus)
    signal_display_ft = QtCore.pyqtSignal(list, list)

    def set_up_signals(self):
        self.signal_display_positions.connect(self.display_position)
        self.signal_display_tool_offset.connect(self.display_tool_offset)
        self.signal_display_ft_offset.connect(self.display_ft_offset)
        self.signal_display_wp_offset.connect(self.display_wp_offset)
        self.signal_mode.connect(self.gui.mode_disp.setText)
        self.signal_status.connect(self.display_status)
        self.signal_gui_lockout_section_enable.connect(self.gui_lockout_section_enable)
        self.signal_current_cmd_number.connect(self.gui.current_cmd_number.setText)
        self.signal_progress_bar.connect(self.gui.progress_bar.setValue)
        self.signal_current_cmd_text.connect(self.gui.current_cmd_text.setPlainText)
        self.signal_toolpath_name.connect(self.gui.toolpath_name.setText)
        self.signal_status_lights.connect(self.display_status_lights)
        self.signal_display_ft.connect(self.display_ft)


    def __init__(self, egm, gui):
        QtCore.QObject.__init__(self)

        self.safety_status = SafetyStatus()
        self.prev_safety_status = SafetyStatus()
        self.last_displayed_safety_status = None

        k = 1.0e5
        self.force_sim = ft_simulation.FTSim([0,0,-k])

        self.egm = egm
        self.gui = gui
        self.tp_ctrl = toolpath_control.ToolpathControl()

        self.is_running = True
        self.mode = "manual_jog"
        self.mode_changed = True
        self.gui.mode_disp.setText(self.mode) # TODO

        self.work_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.display_wp_offset(self.work_offset) # TODO

        self.tool_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.display_tool_offset(self.tool_offset)

        self.ft_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.display_ft_offset(self.ft_offset)

        self.local_robot_position = None
        self.tool_position = None
        self.tool_wp_position = None

        self.toolpath_state = "standby"
        self.toolpath_state_changed = False

        self.set_up_gui_buttons()
        self.set_up_signals()
        
        self.display_DRO = True
        self.DRO_timer = QTimer()
        self.DRO_timer.start(33) # ms
        self.DRO_timer.timeout.connect(self.handle_DRO_timer)

    ##############################################################################
    # Start GUI Functions                                                        #
    ##############################################################################
    def set_up_gui_buttons(self):
        # Set up workpiece offset buttons
        self.gui.wp_os_x.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_y.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_z.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_rx.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_ry.valueChanged.connect(self.update_wp_os)
        self.gui.wp_os_rz.valueChanged.connect(self.update_wp_os)

        # Loading buttons
        self.gui.load_toolpath_button.clicked.connect(self.load_toolpath_dialog)
        self.gui.load_config_button.clicked.connect(self.load_config_dialog)

        # Mode selector
        self.gui.mode_dial.valueChanged.connect(self.handle_mode_change)

        # Start / Stop / Hold buttons
        self.start_clicked = False
        self.stop_clicked = False
        self.hold_clicked = False
        self.gui.start_button.clicked.connect(self.handle_start_button)
        self.gui.stop_button.clicked.connect(self.handle_stop_button)
        self.gui.hold_button.clicked.connect(self.handle_hold_button)

    def display_ft(self, world_ft, wp_ft):
        self.gui.world_fx.display(world_ft[3])
        self.gui.world_fy.display(world_ft[4])
        self.gui.world_fz.display(world_ft[5])
        self.gui.world_tx.display(world_ft[0])
        self.gui.world_ty.display(world_ft[1])
        self.gui.world_tz.display(world_ft[2])

        self.gui.wp_fx.display(wp_ft[3])
        self.gui.wp_fy.display(wp_ft[4])
        self.gui.wp_fz.display(wp_ft[5])
        self.gui.wp_tx.display(wp_ft[0])
        self.gui.wp_ty.display(wp_ft[1])
        self.gui.wp_tz.display(wp_ft[2])

    def gui_lockout_section_enable(self, is_enabled):
        self.gui.mode_dial.setEnabled(is_enabled)
        self.gui.load_toolpath_button.setEnabled(is_enabled)
        self.gui.load_config_button.setEnabled(is_enabled)
        self.gui.wp_tab.setEnabled(is_enabled)


    def display_status_lights(self, safety_status):
        def col(b):
            if b:
                return "background-color: green"
            else:
                return "background-color: red"

        self.gui.egm_connected.setStyleSheet(col(safety_status.egm_connected))
        self.gui.rapid_running.setStyleSheet(col(safety_status.rapid_running))
        self.gui.motors_on.setStyleSheet(col(safety_status.motors_on))

        self.gui.ft_connected.setStyleSheet(col(safety_status.ft_connected))
        self.gui.ft_status_normal.setStyleSheet(col(safety_status.ft_status_normal))

        self.gui.toolpath_loaded.setStyleSheet(col(safety_status.toolpath_loaded))
        self.gui.toolpath_config_loaded.setStyleSheet(col(safety_status.toolpath_config_loaded))
        self.gui.wp_os_loaded.setStyleSheet(col(safety_status.wp_os_loaded))
        self.gui.tool_os_loaded.setStyleSheet(col(safety_status.tool_os_loaded))
        self.gui.ft_os_loaded.setStyleSheet(col(safety_status.ft_os_loaded))
        self.gui.f_below_pos_thresh.setStyleSheet(col(safety_status.f_below_pos_thresh))
        self.gui.f_below_f_ctrl_thresh.setStyleSheet(col(safety_status.f_below_f_ctrl_thresh))

    def display_status(self, status_str, color):
        self.gui.status_disp.setText(status_str)
        self.gui.status_disp.setStyleSheet("QLineEdit {background-color: " + color + ";}")

    def display_tool_offset(self, T):
        rx, ry, rz = rox.R2rpy(T.R)
        self.gui.tool_os_x.setValue(T.p[0])
        self.gui.tool_os_y.setValue(T.p[1])
        self.gui.tool_os_z.setValue(T.p[2])
        self.gui.tool_os_rx.setValue(np.rad2deg(rx))
        self.gui.tool_os_ry.setValue(np.rad2deg(ry))
        self.gui.tool_os_rz.setValue(np.rad2deg(rz))

    def display_ft_offset(self, T):
        rx, ry, rz = rox.R2rpy(T.R)
        self.gui.ft_os_x.setValue(T.p[0])
        self.gui.ft_os_y.setValue(T.p[1])
        self.gui.ft_os_z.setValue(T.p[2])
        self.gui.ft_os_rx.setValue(np.rad2deg(rx))
        self.gui.ft_os_ry.setValue(np.rad2deg(ry))
        self.gui.ft_os_rz.setValue(np.rad2deg(rz))

    def display_wp_offset(self, T):
        self.gui.wp_os_x.blockSignals(True)
        self.gui.wp_os_y.blockSignals(True)
        self.gui.wp_os_z.blockSignals(True)
        self.gui.wp_os_rx.blockSignals(True)
        self.gui.wp_os_ry.blockSignals(True)
        self.gui.wp_os_rz.blockSignals(True)

        rx, ry, rz = rox.R2rpy(T.R)
        self.gui.wp_os_x.setValue(T.p[0])
        self.gui.wp_os_y.setValue(T.p[1])
        self.gui.wp_os_z.setValue(T.p[2])
        self.gui.wp_os_rx.setValue(np.rad2deg(rx))
        self.gui.wp_os_ry.setValue(np.rad2deg(ry))
        self.gui.wp_os_rz.setValue(np.rad2deg(rz))

        self.gui.wp_os_x.blockSignals(False)
        self.gui.wp_os_y.blockSignals(False)
        self.gui.wp_os_z.blockSignals(False)
        self.gui.wp_os_rx.blockSignals(False)
        self.gui.wp_os_ry.blockSignals(False)
        self.gui.wp_os_rz.blockSignals(False)

    def handle_DRO_timer(self): self.display_DRO = True
    def handle_start_button(self): self.start_clicked = True
    def handle_stop_button(self): self.stop_clicked = True
    def handle_hold_button(self): self.hold_clicked = True

    def handle_mode_change(self, mode_value):
        if mode_value == 0:
            self.mode = "simulator"
        elif mode_value == 1:
            self.mode = "toolpath"
        elif mode_value == 2:
            self.mode = "manual_jog"
        elif mode_value == 3:
            self.mode = "zero_force"

        self.mode_changed = True

    def load_toolpath_dialog(self, val):
        file_name = QFileDialog.getOpenFileName(self.gui, "Open Toolpath", filter="Toolpath Files (*.txt *.toolpath)")[0]
        print(file_name)
        if file_name == '':
            return

        with open(file_name) as f:
            lines = f.readlines()
        self.tp_ctrl.load_toolpath(lines)

        self.signal_toolpath_name.emit(file_name)

        self.toolpath_state = "standby" # TODO
        self.toolpath_state_changed = True

        self.safety_status.toolpath_loaded = True

        self.gui.total_cmds.setText(str(len(self.tp_ctrl.commands)))
        self.signal_current_cmd_number.emit(str(self.tp_ctrl.program_counter+1))
        self.signal_progress_bar.emit(round((self.tp_ctrl.program_counter+1) / len(self.tp_ctrl.commands) * 100.0))
        self.signal_current_cmd_text.emit(str(self.tp_ctrl.commands[self.tp_ctrl.program_counter]))

    def load_config_dialog(self, val):
        file_name = QFileDialog.getOpenFileName(self.gui, "Open Configuration File", filter="Configuration Files (*.yaml)")[0]
        print(file_name)
        if file_name != '':
            self.load_config_file(file_name)

    def update_wp_os(self, val):
        self.safety_status.wp_os_loaded = True

        rx = np.deg2rad(self.gui.wp_os_rx.value())
        ry = np.deg2rad(self.gui.wp_os_ry.value())
        rz = np.deg2rad(self.gui.wp_os_rz.value())

        self.work_offset.R = rox.rpy2R([rx, ry, rz])

        self.work_offset.p[0] = self.gui.wp_os_x.value()
        self.work_offset.p[1] = self.gui.wp_os_y.value()
        self.work_offset.p[2] = self.gui.wp_os_z.value()

    def display_position(self, tool_position, tool_wp_position):
        self.gui.world_x.display(tool_position.p[0])
        self.gui.world_y.display(tool_position.p[1])
        self.gui.world_z.display(tool_position.p[2])

        rx, ry, rz = rox.R2rpy(tool_position.R)
        self.gui.world_rx.display(np.rad2deg(rx))
        self.gui.world_ry.display(np.rad2deg(ry))
        self.gui.world_rz.display(np.rad2deg(rz))

        self.gui.wp_x.display(tool_wp_position.p[0])
        self.gui.wp_y.display(tool_wp_position.p[1])
        self.gui.wp_z.display(tool_wp_position.p[2])

        rx, ry, rz = rox.R2rpy(tool_wp_position.R)
        self.gui.wp_rx.display(np.rad2deg(rx))
        self.gui.wp_ry.display(np.rad2deg(ry))
        self.gui.wp_rz.display(np.rad2deg(rz))

    ##############################################################################
    # End GUI Functions                                                          #
    ##############################################################################

    def start_loop(self):
        self.thread = threading.Thread(target = self.state_machine_loop)
        self.thread.start()

    def stop_loop(self):
        self.is_running = False
        self.thread.join()

    def state_machine_loop(self):
        while self.is_running:
            self.step()

    def load_config_file(self, filename):
        with open(filename, 'r') as file:
            config = yaml.safe_load(file)

        if "toolpath_control_params" in config:
            self.tp_ctrl.params = config["toolpath_control_params"]
            print("Loaded toolpath control parameters:")
            print(self.tp_ctrl.params)
            self.safety_status.toolpath_config_loaded = True

        if "work_offset" in config:
            p = config["work_offset"]["pos"]
            print("p:", p)
            R = rox.rpy2R(np.deg2rad(config["work_offset"]["euler_deg"]))
            self.work_offset = rox.Transform(R,p)
            self.signal_display_wp_offset.emit(self.work_offset)
            print("Loaded work offset:")
            print(self.work_offset)
            self.safety_status.wp_os_loaded = True

        if "tool_offset" in config:
            p = config["tool_offset"]["pos"]
            R = rox.rpy2R(np.deg2rad(config["tool_offset"]["euler_deg"]))
            self.tool_offset = rox.Transform(R,p)
            self.signal_display_tool_offset.emit(self.tool_offset)
            print("Loaded tool offset:")
            print(self.tool_offset)
            self.safety_status.tool_os_loaded = True

        if "ft_offset" in config:
            p = config["ft_offset"]["pos"]
            R = rox.rpy2R(np.deg2rad(config["ft_offset"]["euler_deg"]))
            self.ft_offset = rox.Transform(R,p)
            self.signal_display_ft_offset.emit(self.ft_offset)
            print("Loaded force/torque offset:")
            print(self.ft_offset)
            self.safety_status.ft_os_loaded = True

    def step(self):
        self.egm_connected, self.egm_state = self.egm.receive_from_robot(.1)
        if self.egm_connected:
            # Clear queue
            i = 0
            while True:
                egm_connected_i, egm_state_i = self.egm.receive_from_robot()
                if egm_connected_i: # there was another msg waiting
                    self.egm_state = egm_state_i
                    i += 1
                else: # previous msg was end of queue
                    break
            if i > 0:
                print("Warning: Extra msgs in queue: ", i)
        
            fb = self.egm_state.robot_message.feedBack.cartesian
            pos = np.array([fb.pos.x, fb.pos.y, fb.pos.z])/1000.0 # mm -> m
            quat = [fb.orient.u0, fb.orient.u1, fb.orient.u2, fb.orient.u3]
            robot_pos_measured = rox.Transform(rox.q2R(quat), pos)

            tool_position_measured = robot_pos_measured * self.tool_offset
            tool_wp_position_measured = self.work_offset.inv() * tool_position_measured

            if self.display_DRO:
                self.signal_display_positions.emit(tool_position_measured, tool_wp_position_measured)
                self.display_DRO = False
            

        self.prev_safety_status = dataclasses.replace(self.safety_status) # only valid for items set on this thread
        self.safety_status.egm_connected = self.egm_connected
        self.safety_status.rapid_running = self.egm_connected and self.egm_state.rapid_running
        self.safety_status.motors_on = self.egm_connected and self.egm_state.motors_on


        safety_status_disp = dataclasses.replace(self.safety_status)
        if self.last_displayed_safety_status != safety_status_disp:
            self.signal_status_lights.emit(safety_status_disp)
            self.last_displayed_safety_status = safety_status_disp

        if self.safety_status.egm_ok() and not self.prev_safety_status.egm_ok():
            self.local_robot_position = robot_pos_measured

        if self.safety_status.egm_ok():
            self.tool_position = self.local_robot_position * self.tool_offset
            self.tool_wp_position = self.work_offset.inv() * self.tool_position

        if self.mode_changed:
            self.mode_changed = False
            self.signal_mode.emit(self.mode)
            self.toolpath_state_changed = True

        if self.mode == "manual_jog":
            self.mode_manual_jog()
        elif self.mode == "toolpath":
            self.mode_toolpath()
        elif self.mode == "simulator":
            self.mode_toolpath(simulate_force = True, zero_force = False)
        elif self.mode == "zero_force":
            self.mode_toolpath(simulate_force = False, zero_force = True)
        else:
            raise ValueError("Wrong state: " + str(self.state))

        if self.local_robot_position is not None:
            pos = self.local_robot_position.p * 1000.0 # m -> mm
            quat = rox.R2q(self.local_robot_position.R)
            t_1 = time.perf_counter()
            send_res = self.egm.send_to_robot_cart(pos, quat)

    def mode_manual_jog(self):
        if self.safety_status != self.prev_safety_status:
            if self.safety_status.egm_ok():
                self.signal_status.emit("Running", "")
            else:
                self.signal_status.emit("EGM not ready", "red")
                
        if not self.safety_status.egm_ok():
            return

        # Calcualate desired velocity
        jog_linear_rate = 1e-3 * self.gui.jog_linear_rate.value() # mm -> m
        jog_angular_rate = np.deg2rad(self.gui.jog_angular_rate.value())

        v_des = None
        v_rot_des = None

        # Linear
        if self.gui.plus_x.isDown():
            v_des = [1.0, 0.0, 0.0]
        elif self.gui.minus_x.isDown():
            v_des = [-1, 0.0, 0.0]
        elif self.gui.plus_y.isDown():
            v_des = [0.0, 1.0, 0.0]
        elif self.gui.minus_y.isDown():
            v_des = [0.0, -1.0, 0.0]
        elif self.gui.plus_z.isDown():
            v_des = [0.0, 0.0, 1.0]
        elif self.gui.minus_z.isDown(): 
            v_des = [0.0, 0.0, -1.0]
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
                v_des = self.tool_wp_position.R.dot(v_des) 
            if v_rot_des is not None:
                v_rot_des = self.tool_position.R.dot(v_rot_des)
        else:
            raise ValueError("Wrong value for jog reference frame dial: " + str(jog_ref_frame_num))

        # Apply velocity to local robot state
        if v_des is not None:
            self.tool_position.p += v_des*jog_linear_rate*TIMESTEP
        if v_rot_des is not None:
            self.tool_position.R = rox.rot(v_rot_des, jog_angular_rate * TIMESTEP).dot(self.tool_position.R)

        if v_des is not None or v_rot_des is not None:
            self.local_robot_position = self.tool_position * self.tool_offset.inv()
        
        #print((t_1 - t_0) / TIMESTEP * 100, '%')

    def mode_toolpath(self, simulate_force = False, zero_force = False):
        if self.toolpath_state == "standby":
            if self.toolpath_state_changed:
                self.toolpath_state_changed = False
                self.signal_status.emit("Standby", "")
                self.start_clicked = False
                self.signal_gui_lockout_section_enable.emit(True)
                
            if self.start_clicked:
                self.start_clicked = False
                if self.safety_status.toolpath_ready():
                    self.toolpath_state = "running"
                    self.toolpath_state_changed = True
        ##############################################################################
        elif self.toolpath_state == "running":
            if self.toolpath_state_changed:
                self.toolpath_state_changed = False
                self.signal_status.emit("Running", "green")
                self.signal_gui_lockout_section_enable.emit(False)
                self.stop_clicked = False
                self.hold_clicked = False

            if self.stop_clicked or not self.safety_status.toolpath_ready():
                self.toolpath_state = "standby"
                self.toolpath_state_changed = True
                print("Stopped toolpath! stop_clicked: {} safety_status: {}".format(self.stop_clicked, self.safety_status))
                self.stop_clicked = False

            if self.hold_clicked:
                self.hold_clicked = False
                self.toolpath_state = "hold_requested"
                self.toolpath_state_changed = True

            self.run_toopath(simulate_force, zero_force)
        ##############################################################################  
        elif self.toolpath_state == "hold_requested":
            if self.toolpath_state_changed:
                self.toolpath_state_changed = False
                self.signal_status.emit("Hold Requested", "yellow")

            if self.stop_clicked or not self.safety_status.toolpath_ready():
                self.toolpath_state = "standby"
                self.toolpath_state_changed = True
                print("Stopped toolpath! stop_clicked: {} safety_status: {}".format(self.stop_clicked, self.safety_status))
                self.stop_clicked = False

            self.run_toopath(simulate_force, zero_force)
            current_cmd = self.tp_ctrl.commands[self.tp_ctrl.program_counter]
            if type(current_cmd) is self.tp_ctrl.CmdMoveL or  type(current_cmd) is self.tp_ctrl.CmdPosCtrl:
                self.toolpath_state = "standby"
                self.toolpath_state_changed = True
        ############################################################################## 
        else:
            raise ValueError("Wrong value for toolpath_state: ", str(self.toolpath_state))

    def run_toopath(self, simulate_force, zero_force):
        if simulate_force:
            force = self.force_sim.read_ft_streaming(self.tool_wp_position.p)
            #force += np.random.normal(loc=0.0, scale = 5.0, size=(6,))
            self.signal_display_ft.emit(list(force), list(force)) # TODO update based on timer
        else:
            force = [0.,0.,0.,0.,0.,0.] # TODO

        is_done, self.tool_wp_position = self.tp_ctrl.step(self.tool_wp_position, force, disable_f_ctrl = zero_force)

        if is_done:
            self.toolpath_state = "standby"
            self.toolpath_state_changed = True
            self.safety_status.toolpath_loaded = False

        self.tool_position = self.work_offset * self.tool_wp_position
        self.local_robot_position = self.tool_position * self.tool_offset.inv()

        self.signal_current_cmd_number.emit(str(self.tp_ctrl.program_counter+1))
        self.signal_progress_bar.emit(round((self.tp_ctrl.program_counter+1) / len(self.tp_ctrl.commands) * 100.0))
        self.signal_current_cmd_text.emit(str(self.tp_ctrl.commands[self.tp_ctrl.program_counter]))


def main():
    import sys
    sys.path.append('../')
    from rpi_abb_irc5 import rpi_abb_irc5
    from gui import rolling_gui
    from PyQt5.QtWidgets import QApplication
    

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