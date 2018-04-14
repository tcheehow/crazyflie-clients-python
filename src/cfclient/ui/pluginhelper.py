#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
Used for passing objects to tabs and toolboxes.
"""


from cflib.utils.callbacks import Caller
from math import sin, pi


__author__ = 'Bitcraze AB'
__all__ = ['PluginHelper']


class PluginHelper():
    """Used for passing objects to tabs and toolboxes"""

    def __init__(self):
        self.cf = None
        self.cfs = {}
        self.menu = None
        self.logConfigReader = None
        self.referenceHeight = 0.400
        self.hover_input_updated = Caller()
        self.useReferenceHeight = False
        self.inputType = 0
        self.inputTimer = 0.000
        self.sinewaveFrequency = 1.0 # 1s

    def send_hover_setpoint(self, vy, vx, yawrate, height):
        if self.useReferenceHeight:
            if (self.inputType == 0): # Step Input
                self.hover_input_updated.call(vy, vx, yawrate, self.referenceHeight)
            if (self.inputType == 1): # Sine Wave
                self.hover_input_updated.call(vy, vx, yawrate, self.sine_wave_generator())
            if (self.inputType == 2): # Ramp
                self.hover_input_updated.call(vy, vx, yawrate, self.referenceHeight)
            #else:
                #self.hover_input_updated.call(vy, vx, yawrate, self.referenceHeight)
        else:
            self.hover_input_updated.call(vy, vx, yawrate, height)

    def sine_wave_generator(self):
        output = 0.2 * sin(2.00 * pi * self.sinewaveFrequency * self.inputTimer)
        output = output + self.referenceHeight
        self.inputTimer = self.inputTimer + 0.001
        return output


