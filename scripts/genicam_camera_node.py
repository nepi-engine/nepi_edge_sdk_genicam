#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_edge_sdk_genicam
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_genicam
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#
import sys
import time
import math
import rospy
import threading
import cv2

from nepi_edge_sdk_base.idx_sensor_if import ROSIDXSensorIF
from nepi_edge_sdk_genicam.genicam_cam_driver import GENICAM_GENERIC_DRIVER_ID, GenicamCamDriver

class GenicamCameraNode:
    DEFAULT_NODE_NAME = "genicam_camera_node"

    DRIVER_SPECIALIZATION_CONSTRUCTORS = {GENICAM_GENERIC_DRIVER_ID: GenicamCamDriver}

    def __init__(self):
        pass # TODO

    def logDeviceInfo(self):
        pass # TODO

    def createResolutionModeMapping(self):
        pass # TODO

    def createFramerateModeMapping(self):
        pass # TODO

    def setResolutionMode(self, mode):
        pass # TODO
    
    def setFramerateMode(self, mode):
        pass # TODO
    
    def getColorImg(self):
        pass # TODO
    
    def stopColorImg(self):
        pass # TODO
    
    def getBWImg(self):
        pass # TODO
    
    def stopBWImg(self):
        pass # TODO

        
if __name__ == '__main__':
    node = GenicamCameraNode()
