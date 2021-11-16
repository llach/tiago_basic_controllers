import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QThread
from python_qt_binding.QtWidgets import QWidget


from sensor_msgs.msg import JointState
from controller_gui import ControllerGUI

class PluginWrapper(Plugin):

    def __init__(self, context):
        
        super(PluginWrapper, self).__init__(context)
        self.setObjectName(self.name)

        # Create QWidget
        self._widget = QWidget()

        # load widget UI layout
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               self.ui_file)
        loadUi(ui_file, self._widget)

        self._widget.setObjectName(self.name)

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self.name)

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self.cg = ControllerGUI(self.used, self.stopped)

                # joint state helpers
        self.jsr, self.jsl = None, None
        self.rname, self.lname = "gripper_right_finger_joint", "gripper_left_finger_joint"
        self.state = []

        self.jssub = rospy.Subscriber("/joint_states", JointState, callback=self.joint_state_cb)

         # GUI widget shortcuts
        self.lbl_status = self._widget.lbl_status
        self.lbl_right = self._widget.lbl_right
        self.lbl_left = self._widget.lbl_left
        self.lbl_des_r = self._widget.lbl_des_r
        self.lbl_des_l = self._widget.lbl_des_l

        self.sld_right = self._widget.sld_right
        self.sld_left = self._widget.sld_left

        # connect signals
        self.sld_right.valueChanged.connect(self.sliderRightChanged)
        self.sld_left.valueChanged.connect(self.sliderLeftChanged)

        # set label defaults
        if self.cg.initialized:
            self.lbl_status.setText("SUCC")
        else:
            self.lbl_status.setText("ERR")

        self.lbl_des_r.setText("desired right: 0.0")
        self.lbl_des_l.setText("desired left: 0.0")

        for l in [self.lbl_right, self.lbl_left]:
            l.setText("-")


    def joint_state_cb(self, jsmsg):
        # get joint indices
        if self.jsl is None or self.jsr is None:
            for i, n in enumerate(jsmsg.name):
                if n == self.rname:
                    self.jsr = i
                elif n == self.lname:
                    self.jsl = i
        self.state = [jsmsg.position[self.jsr], jsmsg.position[self.jsl]]

        self.lbl_right.setText("right pos: {:.4f}".format(self.state[0]))
        self.lbl_left.setText("left pos: {:.4f}".format(self.state[1]))
        