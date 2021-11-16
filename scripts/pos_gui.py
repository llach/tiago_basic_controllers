#!/usr/bin/env python

import sys

from tiago_basic_controllers.position_gui import PositionGUI
from rqt_gui.main import Main

main = Main(filename='pos_gui')
sys.exit(main.main(standalone='tiago_basic_controllers.position_gui.PositionGUI'))