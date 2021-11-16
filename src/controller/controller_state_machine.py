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
from queue import Empty

from gui import rolling_gui
from controller import toolpath_control, logger
from ft_simulation import ft_simulation
from rpi_abb_irc5 import rpi_abb_irc5
from ft_sensor import rpi_ati_net_ft

import general_robotics_toolbox as rox

from PyQt5.QtWidgets import QFileDialog, QApplication
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer
    

TIMESTEP = 0.004

FT_SENSOR_IP = "192.168.1.200" # TODO
F_POS_THRESH = 20.0 # N
F_F_CTRL_THRESH = 2000.0 # N


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
    signal_total_cmds = QtCore.pyqtSignal(str)

    def set_up_signals(self):
        self.signal_display_positions.connect(self.gui.display_position)
        self.signal_display_tool_offset.connect(self.gui.display_tool_offset)
        self.signal_display_ft_offset.connect(self.gui.display_ft_offset)
        self.signal_display_wp_offset.connect(self.gui.display_wp_offset)
        self.signal_mode.connect(self.gui.mode_disp.setText)
        self.signal_status.connect(self.gui.display_status)
        self.signal_gui_lockout_section_enable.connect(self.gui.lockout_section_enable)
        self.signal_current_cmd_number.connect(self.gui.current_cmd_number.setText)
        self.signal_progress_bar.connect(self.gui.progress_bar.setValue)
        self.signal_current_cmd_text.connect(self.gui.current_cmd_text.setPlainText)
        self.signal_toolpath_name.connect(self.gui.toolpath_name.setText)
        self.signal_status_lights.connect(self.gui.display_status_lights)
        self.signal_display_ft.connect(self.gui.display_ft)
        self.signal_total_cmds.connect(self.gui.total_cmds.setText)


    def __init__(self, egm, gui):
        QtCore.QObject.__init__(self)

        self.gui = gui
        self.set_up_signals()

        self.safety_status = SafetyStatus()
        self.prev_safety_status = None

        k = 717046.0 # N / m
        self.force_sim = ft_simulation.FTSim([0,0,-k])

        self.egm = egm
        self.net_ft = rpi_ati_net_ft.NET_FT(FT_SENSOR_IP)
        self.net_ft.start_streaming()

        self.tp_ctrl = toolpath_control.ToolpathControl()

        self.logger = logger.Logger()

        self.is_running = True
        self.mode = "manual_jog"
        self.signal_mode.emit(self.mode)
        self.signal_status.emit("EGM not ready", "red")

        self.work_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.signal_display_wp_offset.emit(self.work_offset)

        self.tool_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.signal_display_tool_offset.emit(self.tool_offset)

        self.ft_offset = rox.Transform(np.eye(3), [0.0,0.0,0.0])
        self.signal_display_ft_offset.emit(self.ft_offset)

        self.local_robot_position = None
        self.tool_position = None
        self.tool_wp_position = None

        self.toolpath_state = "standby"
        self.prev_toolpath_state = None

        self.display_DRO = True
        self.DRO_timer = QTimer()
        self.DRO_timer.start(33) # ms
        self.DRO_timer.timeout.connect(self.handle_DRO_timer)

    def handle_DRO_timer(self): self.display_DRO = True

    def start_loop(self):
        self.thread = threading.Thread(target = self.state_machine_loop)
        self.thread.start()

    def stop_loop(self):
        self.is_running = False
        #self.thread.join()

    def state_machine_loop(self):
        while self.is_running:
            self.step()
        self.logger.stop_logging()

    def load_toolpath_file(self, file_name):
        with open(file_name) as f:
            lines = f.readlines()

        self.tp_ctrl.load_toolpath(lines)

        self.signal_toolpath_name.emit(file_name)

        self.safety_status.toolpath_loaded = True

        self.signal_total_cmds.emit(str(len(self.tp_ctrl.commands)))
        self.signal_current_cmd_number.emit(str(self.tp_ctrl.program_counter+1))
        self.signal_progress_bar.emit(0)
        self.signal_current_cmd_text.emit(str(self.tp_ctrl.commands[self.tp_ctrl.program_counter]))

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
        self.prev_safety_status = dataclasses.replace(self.safety_status)

        # Receive EGM message
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

        # Update EGM safety status based on new messages    
        self.safety_status.egm_connected = self.egm_connected
        self.safety_status.rapid_running = self.egm_connected and self.egm_state.rapid_running
        self.safety_status.motors_on = self.egm_connected and self.egm_state.motors_on

        # Update robot internal state if EGM started back up
        if self.safety_status.egm_ok() and not self.prev_safety_status.egm_ok():
            self.local_robot_position = robot_pos_measured

        # Update local positions for use by the controller
        # TODO do we need this here?
        if self.safety_status.egm_ok():
            self.tool_position = self.local_robot_position * self.tool_offset
            self.tool_wp_position = self.work_offset.inv() * self.tool_position

        # Receive F/T sensor messages
        # [Tx, Ty, Tz, Fx, Fy, Fz]
        ft_connected, ft, ft_status = self.net_ft.try_read_ft_streaming()

        if ft_connected:
            # Apply lever arm to go from F/T at sensor frame to F/T at tool frame
            # This is still represented in the sensor frame
            torque_sensor = ft[0:2]
            force_sensor = ft[3:5]
            # offset from tool to FT sensor
            T_tool_ft = self.tool_offset.inv() * self.ft_offset
            torque_tool = torque_sensor + rox.hat(T_tool_ft.p).dot(force_sensor)
            force_tool = force_sensor

            # Represent F/T at the tool frame in the world frame
            torque_tool_world_f = self.tool_position.R.T.dot(T_tool_ft.R.T.dot(torque_tool))
            force_tool_world_f = self.tool_position.R.T.dot(T_tool_ft.R.T.dot(force_tool))
            ft_world_f = np.append(torque_tool_world_f, force_tool_world_f)
            
            # Represent F/T at the tool frame in the workpiece frame
            torque_tool_wp_f = self.work_offset.R.dot(torque_tool_world_f)
            force_tool_wp_f = self.work_offset.R.dot(force_tool_world_f)
            ft_wp_f = np.append(torque_tool_wp_f, force_tool_wp_f)
        else:
            ft_world_f = np.array([0.,0.,0.,0.,0.,0.])
            ft_wp_f = np.array([0.,0.,0.,0.,0.,0.])

        # Display F/T sensor reading
        # TODO update based on timer
        self.signal_display_ft.emit(list(ft_world_f), list(ft_wp_f))

        # Updated F/T safety status based on new messages
        self.safety_status.ft_connected = ft_connected
        self.safety_status.ft_status_normal = ft_connected and ft_status == 0
        f_magnitude = np.linalg.norm(ft_wp_f)
        self.safety_status.f_below_pos_thresh = ft_connected and f_magnitude < F_POS_THRESH
        self.safety_status.f_below_f_ctrl_thresh = ft_connected and f_magnitude < F_F_CTRL_THRESH


        # Go through all the GUI queues
        start_clicked = self.gui.start_clicked_q.get_and_clear() is not None
        stop_clicked = self.gui.stop_clicked_q.get_and_clear() is not None
        hold_clicked = self.gui.hold_clicked_q.get_and_clear() is not None
        
        new_mode = self.gui.mode_q.get_and_clear()
        mode_changed = new_mode is not None
        if mode_changed:
            self.mode = new_mode
            self.signal_mode.emit(self.mode)

        new_wp_offset = self.gui.wp_offset_q.get_and_clear()
        if new_wp_offset is not None:
            self.work_offset = new_wp_offset
            self.safety_status.wp_os_loaded = True

        try:
            self.load_config_file(self.gui.config_file_q.get_nowait())
        except Empty:
            pass

        try:
            self.load_toolpath_file(self.gui.toolpath_file_q.get_nowait())
        except Empty:
            pass

        # Run control
        if self.mode == "manual_jog":
            self.mode_manual_jog(mode_changed)
        elif self.mode == "toolpath":
            self.mode_toolpath(mode_changed, start_clicked, stop_clicked, hold_clicked, ft_wp_f)
        elif self.mode == "simulator":
            if self.tool_wp_position is not None:
                ft_simulated = self.force_sim.read_ft_streaming(self.tool_wp_position.p)
            else:
                ft_simulated = np.array([0.,0.,0.,0.,0.,0.])
            self.mode_toolpath(mode_changed, start_clicked, stop_clicked, hold_clicked, ft_simulated)
        elif self.mode == "zero_force":
            self.mode_toolpath(mode_changed, start_clicked, stop_clicked, hold_clicked, ft_wp_f, disable_f_ctrl = True)
        else:
            raise ValueError("Wrong state: " + str(self.state))

        if self.safety_status != self.prev_safety_status:
            self.signal_status_lights.emit(self.safety_status)

        # Send command to robot
        if self.local_robot_position is not None:
            pos = self.local_robot_position.p * 1000.0 # m -> mm
            quat = rox.R2q(self.local_robot_position.R)
            send_res = self.egm.send_to_robot_cart(pos, quat)

        if self.logger.is_logging:
            if self.mode == "simulator":
                ft_log = ft_simulated
            else:
                ft_log = ft_wp_f

            curr_cmd = self.tp_ctrl.commands[self.tp_ctrl.program_counter]
            if type(curr_cmd) is self.tp_ctrl.CmdForceCtrlZ:
                f_des = curr_cmd.fz
            else:
                f_des = 0.0

            if self.egm_state is not None:
                q_log = self.egm_state.joint_angles
                tool_meas_log = tool_wp_position_measured
            else:
                q_log = np.array([0.,0.,0.,0.,0.,0.])
                tool_meas_log = rox.Transform(np.eye(3), [0.0,0.0,0.0])

            self.logger.log(
                q = q_log,
                egm = tool_meas_log,
                cmd = self.tool_wp_position,
                FT = ft_log,
                f_des = f_des,
                run_status = f"{self.mode} - {self.toolpath_state}")

    def mode_manual_jog(self, mode_changed):
        if (self.safety_status != self.prev_safety_status) or mode_changed:
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

    def mode_toolpath(self, mode_changed, start_clicked, stop_clicked, hold_clicked, force, disable_f_ctrl = False):
        toolpath_state_changed = self.prev_toolpath_state != self.toolpath_state
        self.prev_toolpath_state = self.toolpath_state

        if self.toolpath_state == "standby":
            if toolpath_state_changed or mode_changed or self.safety_status != self.prev_safety_status:
                if self.safety_status.toolpath_ready():
                    self.signal_status.emit("Standby - Ready!", "")
                else:
                    self.signal_status.emit("Standby - Not Ready", "")
                self.signal_gui_lockout_section_enable.emit(True)
                
            elif start_clicked:
                if self.safety_status.toolpath_ready():
                    self.toolpath_state = "running"


        elif self.toolpath_state == "running":
            if toolpath_state_changed:
                self.signal_status.emit("Running", "green")
                self.signal_gui_lockout_section_enable.emit(False)
                self.logger.start_logging("log.csv") # TODO filename

            elif stop_clicked or not self.safety_status.toolpath_ready():
                self.toolpath_state = "standby"
                print("Stopped toolpath! stop_clicked: {} safety_status: {}".format(stop_clicked, self.safety_status))

            elif hold_clicked:
                self.toolpath_state = "hold_requested"

            self.run_toopath(force, disable_f_ctrl)   


        elif self.toolpath_state == "hold_requested":
            if toolpath_state_changed:
                self.signal_status.emit("Hold Requested", "yellow")

            if stop_clicked or not self.safety_status.toolpath_ready():
                self.toolpath_state = "standby"
                print("Stopped toolpath! stop_clicked: {} safety_status: {}".format(stop_clicked, self.safety_status))

            self.run_toopath(force, disable_f_ctrl)
            current_cmd = self.tp_ctrl.commands[self.tp_ctrl.program_counter]
            if type(current_cmd) is self.tp_ctrl.CmdMoveL or  type(current_cmd) is self.tp_ctrl.CmdPosCtrl:
                self.toolpath_state = "standby"
        
        else:
            raise ValueError("Wrong value for toolpath_state: ", str(self.toolpath_state))

        

    def run_toopath(self, force, disable_f_ctrl):
        is_done, self.tool_wp_position = self.tp_ctrl.step(self.tool_wp_position, force, disable_f_ctrl = disable_f_ctrl)

        if is_done:
            self.toolpath_state = "standby"
            self.safety_status.toolpath_loaded = False
            self.logger.stop_logging()

        self.tool_position = self.work_offset * self.tool_wp_position
        self.local_robot_position = self.tool_position * self.tool_offset.inv()

        self.signal_current_cmd_number.emit(str(self.tp_ctrl.program_counter+1))
        self.signal_progress_bar.emit(round((self.tp_ctrl.program_counter+1) / len(self.tp_ctrl.commands) * 100.0))
        self.signal_current_cmd_text.emit(str(self.tp_ctrl.commands[self.tp_ctrl.program_counter]))

def main():
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