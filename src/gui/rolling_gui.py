import sys
import os

from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow
from PyQt5.uic import loadUi

from queue import Queue, Empty

import general_robotics_toolbox as rox
import numpy as np


class ClearQueue(Queue):
    def get_and_clear(self):
        res = None
        try:
            while True:
                res = super().get_nowait()
        except Empty:
            return res

class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        abspath = os.path.join(os.path.dirname(os.path.realpath(__file__)), "rolling_gui.ui")
        loadUi(abspath, self)

        # Only need to check if queue nonempty
        self.start_clicked_q = ClearQueue()
        self.stop_clicked_q = ClearQueue()
        self.hold_clicked_q = ClearQueue()

        self.tare_q = ClearQueue()

        # Only need to check most recent
        self.mode_q = ClearQueue()
        self.wp_offset_q = ClearQueue()

        # Regular queue - need to read every file
        self.config_file_q = Queue()
        self.toolpath_file_q = Queue()

        self.set_up_gui_buttons()

    def set_up_gui_buttons(self):
        # Set up workpiece offset buttons
        self.wp_os_x.valueChanged.connect(self.send_wp_os)
        self.wp_os_y.valueChanged.connect(self.send_wp_os)
        self.wp_os_z.valueChanged.connect(self.send_wp_os)
        self.wp_os_rx.valueChanged.connect(self.send_wp_os)
        self.wp_os_ry.valueChanged.connect(self.send_wp_os)
        self.wp_os_rz.valueChanged.connect(self.send_wp_os)

        # Loading buttons
        self.load_toolpath_button.clicked.connect(self.load_toolpath_dialog)
        self.load_config_button.clicked.connect(self.load_config_dialog)

        # Mode selector
        self.mode_dial.valueChanged.connect(self.handle_mode_change)

        # Start / Stop / Hold buttons
        self.start_button.clicked.connect(self.handle_start_button)
        self.stop_button.clicked.connect(self.handle_stop_button)
        self.hold_button.clicked.connect(self.handle_hold_button)

        self.tare_button.clicked.connect(self.handle_tare_button)

    ##############################################################################
    # Handle user input                                                          #
    ##############################################################################
    def load_toolpath_dialog(self, val):
        file_name = QFileDialog.getOpenFileName(self, "Open Toolpath", filter="Toolpath Files (*.txt *.toolpath)")[0]
        print(file_name)
        if file_name != '':
            self.toolpath_file_q.put(file_name)

    def load_config_dialog(self, val):
        file_name = QFileDialog.getOpenFileName(self, "Open Configuration File", filter="Configuration Files (*.yaml)")[0]
        print(file_name)
        if file_name != '':
            self.config_file_q.put(file_name)

    def handle_start_button(self): self.start_clicked_q.put(True)
    def handle_stop_button(self): self.stop_clicked_q.put(True)
    def handle_hold_button(self): self.hold_clicked_q.put(True)
    def handle_tare_button(self): self.tare_q.put(True)

    def handle_mode_change(self, mode_value):
        if mode_value == 0:
            mode_name = "simulator"
        elif mode_value == 1:
            mode_name = "toolpath"
        elif mode_value == 2:
            mode_name = "manual_jog"
        elif mode_value == 3:
            mode_name = "zero_force"
        self.mode_q.put(mode_name)

    def send_wp_os(self, val):
        rx = np.deg2rad(self.wp_os_rx.value())
        ry = np.deg2rad(self.wp_os_ry.value())
        rz = np.deg2rad(self.wp_os_rz.value())
        R = rox.rpy2R([rx, ry, rz])

        p = [self.wp_os_x.value(), self.wp_os_y.value(), self.wp_os_z.value()]

        self.wp_offset_q.put(rox.Transform(R,p))

    ##############################################################################
    # Display things                                                             #
    ##############################################################################

    def lockout_section_enable(self, is_enabled):
        self.mode_dial.setEnabled(is_enabled)
        self.load_toolpath_button.setEnabled(is_enabled)
        self.load_config_button.setEnabled(is_enabled)
        self.wp_tab.setEnabled(is_enabled)

    def display_status(self, status_str, color):
        self.status_disp.setText(status_str)
        self.status_disp.setStyleSheet("QLineEdit {background-color: " + color + ";}")

    def display_ft(self, world_ft, wp_ft):
        self.world_fx.display(world_ft[3])
        self.world_fy.display(world_ft[4])
        self.world_fz.display(world_ft[5])
        self.world_tx.display(world_ft[0])
        self.world_ty.display(world_ft[1])
        self.world_tz.display(world_ft[2])

        self.wp_fx.display(wp_ft[3])
        self.wp_fy.display(wp_ft[4])
        self.wp_fz.display(wp_ft[5])
        self.wp_tx.display(wp_ft[0])
        self.wp_ty.display(wp_ft[1])
        self.wp_tz.display(wp_ft[2])

    def display_position(self, tool_position, tool_wp_position):
        self.world_x.display(tool_position.p[0])
        self.world_y.display(tool_position.p[1])
        self.world_z.display(tool_position.p[2])

        rx, ry, rz = rox.R2rpy(tool_position.R)
        self.world_rx.display(np.rad2deg(rx))
        self.world_ry.display(np.rad2deg(ry))
        self.world_rz.display(np.rad2deg(rz))

        self.wp_x.display(tool_wp_position.p[0])
        self.wp_y.display(tool_wp_position.p[1])
        self.wp_z.display(tool_wp_position.p[2])

        rx, ry, rz = rox.R2rpy(tool_wp_position.R)
        self.wp_rx.display(np.rad2deg(rx))
        self.wp_ry.display(np.rad2deg(ry))
        self.wp_rz.display(np.rad2deg(rz))

    def display_status_lights(self, safety_status):
        def col(b):
            if b:
                return "background-color: green"
            else:
                return "background-color: red"

        self.egm_connected.setStyleSheet(col(safety_status.egm_connected))
        self.rapid_running.setStyleSheet(col(safety_status.rapid_running))
        self.motors_on.setStyleSheet(col(safety_status.motors_on))

        self.ft_connected.setStyleSheet(col(safety_status.ft_connected))
        self.ft_status_normal.setStyleSheet(col(safety_status.ft_status_normal))

        self.toolpath_loaded.setStyleSheet(col(safety_status.toolpath_loaded))
        self.toolpath_config_loaded.setStyleSheet(col(safety_status.toolpath_config_loaded))
        self.wp_os_loaded.setStyleSheet(col(safety_status.wp_os_loaded))
        self.tool_os_loaded.setStyleSheet(col(safety_status.tool_os_loaded))
        self.ft_os_loaded.setStyleSheet(col(safety_status.ft_os_loaded))
        self.f_below_pos_thresh.setStyleSheet(col(safety_status.f_below_pos_thresh))
        self.f_below_f_ctrl_thresh.setStyleSheet(col(safety_status.f_below_f_ctrl_thresh))

    def display_ft_offset(self, T):
        rx, ry, rz = rox.R2rpy(T.R)
        self.ft_os_x.setValue(T.p[0])
        self.ft_os_y.setValue(T.p[1])
        self.ft_os_z.setValue(T.p[2])
        self.ft_os_rx.setValue(np.rad2deg(rx))
        self.ft_os_ry.setValue(np.rad2deg(ry))
        self.ft_os_rz.setValue(np.rad2deg(rz))

    def display_tool_offset(self, T):
        rx, ry, rz = rox.R2rpy(T.R)
        self.tool_os_x.setValue(T.p[0])
        self.tool_os_y.setValue(T.p[1])
        self.tool_os_z.setValue(T.p[2])
        self.tool_os_rx.setValue(np.rad2deg(rx))
        self.tool_os_ry.setValue(np.rad2deg(ry))
        self.tool_os_rz.setValue(np.rad2deg(rz))

    def display_wp_offset(self, T):
        self.wp_os_x.blockSignals(True)
        self.wp_os_y.blockSignals(True)
        self.wp_os_z.blockSignals(True)
        self.wp_os_rx.blockSignals(True)
        self.wp_os_ry.blockSignals(True)
        self.wp_os_rz.blockSignals(True)

        rx, ry, rz = rox.R2rpy(T.R)
        self.wp_os_x.setValue(T.p[0])
        self.wp_os_y.setValue(T.p[1])
        self.wp_os_z.setValue(T.p[2])
        self.wp_os_rx.setValue(np.rad2deg(rx))
        self.wp_os_ry.setValue(np.rad2deg(ry))
        self.wp_os_rz.setValue(np.rad2deg(rz))

        self.wp_os_x.blockSignals(False)
        self.wp_os_y.blockSignals(False)
        self.wp_os_z.blockSignals(False)
        self.wp_os_rx.blockSignals(False)
        self.wp_os_ry.blockSignals(False)
        self.wp_os_rz.blockSignals(False)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()

    win.show()

    sys.exit(app.exec())