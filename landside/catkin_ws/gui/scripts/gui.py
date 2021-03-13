#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))