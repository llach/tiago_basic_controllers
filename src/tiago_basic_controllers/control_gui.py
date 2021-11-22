import rospy

from python_qt_binding.QtCore import QThread
from plugin_wrapper import PluginWrapper

from std_msgs.msg import Float64

class PubThread(QThread):

    def __init__(self,  rsld, lsld, rate=240):
        super(QThread, self).__init__()
        self.rsld = rsld
        self.lsld = lsld
        self.rate = rospy.Rate(rate)

        self.rpub = rospy.Publisher("/gripper_right_finger_position_controller/command", Float64, queue_size=1)
        self.lpub = rospy.Publisher("/gripper_left_finger_position_controller/command", Float64, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            rv = self.rsld.value() / 1000.
            lv = self.lsld.value() / 1000.

            self.rpub.publish(rv)
            self.lpub.publish(lv)
            self.rate.sleep()


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

        self.pt = PubThread(self.sld_pos_right, self.sld_pos_left)
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