#!/usr/bin/env python

import sys

from tiago_basic_controllers.control_gui import ControlGUI
from rqt_gui.main import Main

main = Main(filename='cont_gui')
sys.exit(main.main(standalone='tiago_basic_controllers.control_gui.ControlGUI'))