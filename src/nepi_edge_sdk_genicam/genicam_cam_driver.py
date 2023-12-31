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
import sys, subprocess
import cv2
import threading, time

# TODO: replace
GENICAM_GENERIC_DRIVER_ID = 'GenericGenicam'

class GenicamCamDriver(object):
    MAX_CONSEC_FRAME_FAIL_COUNT = 3

    def __init__(self, device_path):
        pass # TODO

    def isConnected(self):
        pass # TODO

    def initCameraControlsDict(self):
        pass # TODO

    def initVideoFormatDict(self):
        pass # TODO

    def getAvailableScaledCameraControls(self):
        pass # TODO

    def getAvailableDiscreteCameraControls(self):
        pass # TODO

    def setScaledCameraControl(self, genicam_setting_name, scaled_val):
        pass # TODO

    def getScaledCameraControl(self, genicam_setting_name):
        pass # TODO

    def hasAdjustableCameraControl(self, genicam_setting_name):
        pass # TODO

    def hasAdjustableResolution(self):
        pass # TODO

    def hasAdjustableFramerate(self):
        pass # TODO

    def getCurrentVideoSettings(self):
        pass # TODO

    def getCurrentResolution(self):
        pass # TODO

    def setResolution(self, resolution_dict):
        pass # TODO

    def getCurrentResolutionAvailableFramerates(self):
        pass # TODO

    def setFramerate(self, max_fps):
        pass # TODO

    def getFramerate(self):
        pass # TODO

    def getCurrentFormat(self):
        pass # TODO

    def getCurrentFormatAvailableResolutions(self):
        pass # TODO

    def imageAcquisitionRunning(self):
        pass # TODO

    def startImageAcquisition(self):
        pass # TODO

    def runImgAcqThread(self):
        pass # TODO

    def stopImageAcquisition(self, hold_lock=False):
        pass # TODO

    def getImage(self):
        pass # TODO

if __name__ == '__main__':
    pass # TODO
