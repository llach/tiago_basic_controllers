import rospy
import numpy as np

from python_qt_binding.QtCore import QThread
from plugin_wrapper import PluginWrapper

from simple_pid import PID
from std_msgs.msg import Float64, Float64MultiArray

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

        self.integral_limits = (-0.005, 0.005)
        self.pid_right.output_limits = self.integral_limits
        self.pid_left.output_limits = self.integral_limits

        self.set_tunings()

        self.rpub = rospy.Publisher("/gripper_right_finger_position_controller/command", Float64, queue_size=1)
        self.lpub = rospy.Publisher("/gripper_left_finger_position_controller/command", Float64, queue_size=1)
        
        self.dppub = rospy.Publisher("/desired_pos", Float64MultiArray, queue_size=1)
        self.dvpub = rospy.Publisher("/desired_vel", Float64MultiArray, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            self.set_tunings()

            if self.cg.cmode == "pos":
                rv = self.rpsld.value() / 1000.
                lv = self.lpsld.value() / 1000.

                self.dvpub.publish(Float64MultiArray(data=self.cg.current_vel))
                self.dppub.publish(Float64MultiArray(data=[rv, lv]))
            elif self.cg.cmode == "vel":
                des_vr = self.rvsld.value() /100.
                des_vl = self.lvsld.value() /100.

                vr_err = ((self.cg.current_vel[0]) - des_vr)*self.dt
                vl_err = ((self.cg.current_vel[1]) - des_vl)*self.dt

                qdot_r = self.pid_right(vr_err)
                qdot_l = self.pid_left(vl_err)

                rv = self.cg.current_pos[0] + qdot_r
                lv = self.cg.current_pos[1] + qdot_l

                self.dvpub.publish(Float64MultiArray(data=[des_vr, des_vl]))
                self.dppub.publish(Float64MultiArray(data=self.cg.current_pos))
            else:
                print("unkown control mode {}".format(self.cg.cmode))

            self.rpub.publish(rv)
            self.lpub.publish(lv)
            self.r.sleep()

    def set_tunings(self):
        self.pid_right.tunings = self.cg.pid_tunings
        self.pid_left.tunings = self.cg.pid_tunings


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

        self.sld_vel_right.sliderReleased.connect(self.velSliderRightReleased)
        self.sld_vel_left.sliderReleased.connect(self.velSliderLeftReleased)

        self.spin_p.valueChanged.connect(self.update_pid_tunings)
        self.spin_i.valueChanged.connect(self.update_pid_tunings)
        self.spin_d.valueChanged.connect(self.update_pid_tunings)

        self.pid_tunings = [0.0, 0.0, 0.0]
        self.update_pid_tunings()

        self.pt = PubThread(self.sld_pos_right, self.sld_pos_left, self.sld_vel_right, self.sld_vel_left, self)
        self.pt.start()
        
    def velSliderRightChanged(self):
        v = self.sld_vel_right.value() / 100.
        self.lbl_des_r.setText("desired right: {:.3f}".format(v))

    def velSliderLeftChanged(self):
        v = self.sld_vel_left.value() / 100.
        self.lbl_des_l.setText("desired left: {:.3f}".format(v))

    def velSliderRightReleased(self):
        self.sld_vel_right.setValue(0.0)
    
    def velSliderLeftReleased(self):
        self.sld_vel_left.setValue(0.0)

    def posSliderRightChanged(self):
        v = self.sld_pos_right.value() / 1000.
        self.lbl_des_r.setText("desired right: {:.4f}".format(v))

    def posSliderLeftChanged(self):
        v = self.sld_pos_left.value() / 1000.
        self.lbl_des_l.setText("desired left: {:.4f}".format(v))

    def update_pid_tunings(self):
        self.pid_tunings = [self.spin_p.value(), self.spin_i.value(), self.spin_d.value()]
        print("new PID tunings {}".format(self.pid_tunings))