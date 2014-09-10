#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_computer_admin')
from faa_computer_admin import control
control.cli()

