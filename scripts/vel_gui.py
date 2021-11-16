#!/usr/bin/env python

import sys

from tiago_basic_controllers.velocity_gui import VelocityGUI
from rqt_gui.main import Main

main = Main(filename='vel_gui')
sys.exit(main.main(standalone='tiago_basic_controllers.velocity_gui.VelocityGUI'))