import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QThread
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from controller_manager.controller_manager_interface import *
from controller_manager_msgs.srv import ListControllers


class ControllerGUI(object):

    def __init__(self):
        self.lc = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        self.cons = None

        try:
            self.setup_controllers()
        except Exception as e:
            print("could not connect to CM service: {}".format(e))

    def setup_controllers(self):
        print("initializing controllers ...")

        self.get_controllers()
        print("available controllers:")
        for c in self.cons:
            print("{} - {}".format(c.name, c.state))

        print("stopping other controllers ...")
        self.stop_others()

        print("starting our controllers ...")
        self.start_used()

        print("done")

    def start_used(self):
        for c in self.used:
            if not self.is_controller_loaded(c):
                load_controller(c)
            if not self.is_controller_running(c):
                start_controller(c)
    
    def stop_others(self):
        for c in self.stopped:
            if self.is_controller_running(c):
                print("stopping {}".format(c))
                stop_controller(c)

    def get_controllers(self):
        print("getting controllers ...")
        self.cons = self.lc().controller

        if self.cons == []:
            print("no controllers loaded!!")
            self.state_lbl.setText("ERR")
            self.cons = None
            return False
        
        return True

    def is_controller_loaded(self, name):
        for c in self.cons:
            if c.name == name:
                return True
        return False

    def is_controller_running(self, name):
        for c in self.cons:
            if c.name == name and c.state == 'running':
                return True
        return False

class PositionGUI(Plugin, ControllerGUI):

    def __init__(self, context):
        self.used = [
            "gripper_left_finger_position_controller",
            "gripper_right_finger_position_controller"
        ]

        self.stopped = [
            "gripper_controller",
            "gripper_force_controller",
            "gripper_left_finger_velocity_controller",
            "gripper_right_finger_velocity_controller"
        ]

        super(PositionGUI, self).__init__(context)
        self.setObjectName('PositionGUI')

        # Create QWidget
        self._widget = QWidget()

        # load widget UI layout
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'pos.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('PositionGUI')

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # GUI widget shortcuts
        self.lbl_status = self._widget.lbl_status
        self.lbl_right = self._widget.lbl_right
        self.lbl_left = self._widget.lbl_left
        self.lbl_des_r = self._widget.lbl_des_r
        self.lbl_des_l = self._widget.lbl_des_l

        for l in [self.lbl_status, self.lbl_right, self.lbl_left, self.lbl_des_r, self.lbl_des_l]:
            l.setText("-")


