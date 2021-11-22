import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


from sensor_msgs.msg import JointState
from cm_interface import ControllerManagerInterface


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
        
        self.cg = ControllerManagerInterface(self.used, self.stopped)

                # joint state helpers
        self.jsr, self.jsl = None, None
        self.rname, self.lname = "gripper_right_finger_joint", "gripper_left_finger_joint"
        self.state = []

        self.jssub = rospy.Subscriber("/joint_states", JointState, callback=self.joint_state_cb)

         # GUI widget shortcuts
        self.lbl_title = self._widget.lbl_title
        self.lbl_status = self._widget.lbl_status
        self.lbl_right = self._widget.lbl_right
        self.lbl_left = self._widget.lbl_left
        self.lbl_des_r = self._widget.lbl_des_r
        self.lbl_des_l = self._widget.lbl_des_l

        self.rb_pos = self._widget.rb_pos
        self.rb_vel = self._widget.rb_vel

        self.sld_pos_right = self._widget.sld_pos_right
        self.sld_pos_left = self._widget.sld_pos_left

        self.sld_vel_right = self._widget.sld_vel_right
        self.sld_vel_left = self._widget.sld_vel_left

        # connect signals
        self.sld_pos_right.valueChanged.connect(self.posSliderRightChanged)
        self.sld_pos_left.valueChanged.connect(self.posSliderLeftChanged)

        self.rb_pos.toggled.connect(lambda:self.btnstate("pos"))
        self.rb_vel.toggled.connect(lambda:self.btnstate("vel"))

        # set label defaults
        if self.cg.initialized:
            self.lbl_status.setText("SUCC")
        else:
            self.lbl_status.setText("ERR")

        self.lbl_des_r.setText("desired right: 0.0")
        self.lbl_des_l.setText("desired left: 0.0")

        for l in [self.lbl_right, self.lbl_left]:
            l.setText("-")

        # radio btn setup
        self.rb_pos.setChecked(True)
        self.rb_vel.setChecked(False)

        self.sld_vel_right.setEnabled(False)
        self.sld_vel_left.setEnabled(False)

    def joint_state_cb(self, jsmsg):
        # get joint indices
        if self.jsl is None or self.jsr is None:
            for i, n in enumerate(jsmsg.name):
                if n == self.rname:
                    self.jsr = i
                elif n == self.lname:
                    self.jsl = i
            self.set_pos_sliders([jsmsg.position[self.jsr], jsmsg.position[self.jsl]])
        self.state = [jsmsg.position[self.jsr], jsmsg.position[self.jsl]]

        self.lbl_right.setText("right pos: {:.4f}".format(self.state[0]))
        self.lbl_left.setText("left pos: {:.4f}".format(self.state[1]))

    def set_pos_sliders(self, pos=None):
        if pos == None:
            pos = self.state
        self.sld_pos_right.setValue(pos[0])
        self.sld_pos_left.setValue(pos[1])

    def btnstate(self, m):
        self.cmode = m
        if m == "pos":
            self.sld_pos_right.setEnabled(True)
            self.sld_pos_left.setEnabled(True)

            self.sld_vel_right.setEnabled(False)
            self.sld_vel_left.setEnabled(False)
        elif m == "vel":
            self.sld_pos_right.setEnabled(False)
            self.sld_pos_left.setEnabled(False)

            self.sld_vel_right.setEnabled(True)
            self.sld_vel_left.setEnabled(True)