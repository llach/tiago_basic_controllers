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

        self.rpub = rospy.Publisher("/gripper_right_finger_position_controller/command", Float64)
        self.lpub = rospy.Publisher("/gripper_left_finger_position_controller/command", Float64)

    def run(self):
        while not rospy.is_shutdown():
            rv = self.rsld.value() / 1000.
            lv = self.lsld.value() / 1000.

            self.rpub.publish(rv)
            self.lpub.publish(lv)
            self.rate.sleep()


class PositionGUI(PluginWrapper):

    def __init__(self, context):
        self.ui_file = 'pos.ui'
        self.name = 'PositionGUI'

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
        
        super(PositionGUI, self).__init__(context)

        self.pt = PubThread(self.sld_right, self.sld_left)
        self.pt.start()

    def sliderRightChanged(self):
        v = self.sld_right.value() / 1000.
        self.lbl_des_r.setText("desired right: {:.2f}".format(v))

    def sliderLeftChanged(self):
        v = self.sld_left.value() / 1000.
        self.lbl_des_l.setText("desired left: {:.2f}".format(v))