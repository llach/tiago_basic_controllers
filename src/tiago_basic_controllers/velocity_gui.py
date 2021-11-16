from plugin_wrapper import PluginWrapper


class VelocityGUI(PluginWrapper):

    def __init__(self, context):
        self.ui_file = 'vel.ui'
        self.name = 'VelocityGUI'

        print("velocityu")

        # controller manager
        self.used = [
            "gripper_left_finger_velocity_controller",
            "gripper_right_finger_velocity_controller"
        ]

        self.stopped = [
            "gripper_controller",
            "gripper_force_controller",
            "gripper_left_finger_position_controller",
            "gripper_right_finger_position_controller"
        ]
        
        super(VelocityGUI, self).__init__(context)

        self.sld_right.sliderReleased.connect(self.sliderRightReleased)
        self.sld_left.sliderReleased.connect(self.sliderLeftReleased)

    def sliderRightChanged(self):
        v = self.sld_right.value() / 100.
        self.lbl_des_r.setText("desired right: {:.2f}".format(v))

    def sliderLeftChanged(self):
        v = self.sld_left.value() / 100.
        self.lbl_des_l.setText("desired left: {:.2f}".format(v))

    def sliderRightReleased(self):
        self.sld_right.setValue(0.0)
    
    def sliderLeftReleased(self):
        self.sld_left.setValue(0.0)