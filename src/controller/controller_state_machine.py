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

class ControllerStateMachine():
    def __init__(self, egm, gui):
        self.egm = egm
        self.gui = gui

        self.is_running = True
        self.state = "disconnected"
        self.gui.status_disp.setText(self.state)

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
            for i in range(100):
                    self.egm.receive_from_robot() # clear buffer
            self.state = "manual_jog"
            self.gui.status_disp.setText(self.state)
        else:
            pass

    def state_manual_jog(self):
        res, state = self.egm.receive_from_robot(.1)
        if res:
            q_meas = np.deg2rad(state.joint_angles)
            
            T_meas = kin.forkin(q_meas)

            self.gui.world_x.display(T_meas.p[0])
            self.gui.world_y.display(T_meas.p[1])
            self.gui.world_z.display(T_meas.p[2])
            
            self.egm.send_to_robot(q_meas)
        else:
            self.state = "disconnected"
            self.gui.status_disp.setText(self.state)



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