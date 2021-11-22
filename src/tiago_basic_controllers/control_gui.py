import rospy
import numpy as np

from python_qt_binding.QtCore import QThread
from plugin_wrapper import PluginWrapper

from simple_pid import PID
from std_msgs.msg import Float64

class PubThread(QThread):

    def __init__(self,  rpsld, lpsld, rvsld, lvsld, cg, rate=240):
        super(QThread, self).__init__()
        self.r = rospy.Rate(rate)
        self.dt = 1./rate

        self.cg = cg
        self.rate = rate

        self.rpsld = rpsld
        self.lpsld = lpsld

        self.rvsld = rvsld
        self.lvsld = lvsld

        self.pid_right = PID()
        self.pid_left = PID()
        self.tunings = (1, 0.01, 0.0)

        self.pid_right.tunings = self.tunings
        self.pid_left.tunings = self.tunings

        self.rpub = rospy.Publisher("/gripper_right_finger_position_controller/command", Float64, queue_size=1)
        self.lpub = rospy.Publisher("/gripper_left_finger_position_controller/command", Float64, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            if self.cg.cmode == "pos":
                rv = self.rpsld.value() / 1000.
                lv = self.lpsld.value() / 1000.
            elif self.cg.cmode == "vel":
                des_vr = self.rvsld.value() 
                des_vl = self.lvsld.value() 

                vr_err = ((self.current_vel[0]/100.) - des_vr)*self.dt
                vl_err = ((self.current_vel[1]/100.) - des_vl)*self.dt

                rv = self.cg.current_pos[0] + self.pid_right(vr_err)
                lv = self.cg.current_pos[1] + self.pid_left(vl_err)
            else:
                print("unkown control mode {}".format(self.cg.cmode))

            self.rpub.publish(rv)
            self.lpub.publish(lv)
            self.r.sleep()


class ControlGUI(PluginWrapper):

    def __init__(self, context):
        self.ui_file = 'control_gui.ui'
        self.name = "ControlGUI"

        # controller manager
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
        
        super(ControlGUI, self).__init__(context)

        self.pt = PubThread(self.sld_pos_right, self.sld_pos_left, self.sld_vel_right, self.sld_vel_left, self)
        self.pt.start()

        self.sld_vel_right.sliderReleased.connect(self.velSliderRightReleased)
        self.sld_vel_left.sliderReleased.connect(self.velSliderLeftReleased)

    def velSliderRightChanged(self):
        v = self.sld_vel_right.value() / 100.

    def velSliderLeftChanged(self):
        v = self.sld_vel_left.value() / 100.

    def velSliderRightReleased(self):
        self.sld_vel_right.setValue(0.0)
    
    def velSliderLeftReleased(self):
        self.sld_vel_left.setValue(0.0)

    def posSliderRightChanged(self):
        v = self.sld_pos_right.value() / 1000.
        self.lbl_des_r.setText("desired right: {:.2f}".format(v))

    def posSliderLeftChanged(self):
        v = self.sld_pos_left.value() / 1000.
        self.lbl_des_l.setText("desired left: {:.2f}".format(v))