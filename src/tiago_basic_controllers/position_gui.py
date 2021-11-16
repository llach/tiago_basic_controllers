from qt_gui.plugin import Plugin

from controller_manager.controller_manager_interface import *
from controller_manager_msgs.srv import ListControllers


class ControllerGUI(object):

    def __init__(self, *args, **kwargs):
        print("cg init")
        self.lc = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)

class PositionGUI(Plugin, ControllerGUI):

    def __init__(self, context):
        super(Plugin, self).__init__(context)
        super(ControllerGUI, self).__init__()
