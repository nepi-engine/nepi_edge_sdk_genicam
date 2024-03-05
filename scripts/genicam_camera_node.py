#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
        rospy.init_node(self.DEFAULT_NODE_NAME)
        rospy.loginfo(f"Starting {rospy.get_name()}")
        self.node_name = rospy.get_name().split("/")[-1]

        model = rospy.get_param("~model", default=None)
        serial_number = rospy.get_param("~serial_number", default=None)

        # NOTE: In idx_sensor_mgr.py we have to prefix serial_number with "sn" so that it's treated
        #       as a string parameter. Otherwise it's treated automatically as an int and it can
        #       (does) overflow.
        serial_number = serial_number[2:]

        # TODO: add support for other GenTL producers?

        self.driver_id = rospy.get_param("~driver_id", GENICAM_GENERIC_DRIVER_ID)
        rospy.set_param("~driver_id", self.driver_id)
        if self.driver_id not in self.DRIVER_SPECIALIZATION_CONSTRUCTORS:
            rospy.logerr(f"{self.node_name}: unknown driver_id {self.driver_id}")
            return
        DriverConstructor = self.DRIVER_SPECIALIZATION_CONSTRUCTORS[self.driver_id]

        # Establish any user-defined parameters mappings from config file. These are of the form
        #    {""}
        genicam_cfg_file_mappings = rospy.get_param("~genicam_mappings", {})
        
        rospy.loginfo(f"{self.node_name}: Launching {self.driver_id} driver")
        try:
            self.driver = DriverConstructor(model=model, serial_number=serial_number, param_mapping_overrides=genicam_cfg_file_mappings)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to instantiate driver - ({e})")
            sys.exit(-1)

        if not self.driver.isConnected():
            rospy.logerr(f"{self.node_name}: Failed to connect to camera device")

        rospy.loginfo(f"{self.node_name}: ... Connected!")

        self.createResolutionModeMapping()
        self.createFramerateModeMapping()

        idx_callback_names = {
            "Controls": {
                "Resolution": self.setResolutionMode\
                        if self.driver.hasAdjustableResolution()\
                        else None,
                "Framerate": self.setFramerateMode\
                        if self.driver.hasAdjustableFramerate()\
                        else None,
                "Contrast": self.setContrastRatio\
                        if self.driver.hasAdjustableRatioSetting("contrast")\
                        else None,
                "Brightness": self.setBrightnessRatio\
                        if self.driver.hasAdjustableRatioSetting("brightness")\
                        else None,
                "Thresholding": self.setThresholdingRatio\
                        if self.driver.hasAdjustableRatioSetting("thresholding")\
                        else None,                
                "Range": None,
            },
            "Data": {
                "Color2DImg": self.getColorImg,
                "StopColor2DImg": self.stopColorImg,
                "BW2DImg": self.getBWImg,
                "StopBW2DImg": self.stopBWImg,
                "DepthMap": None,  # TODO?
                "StopDepthMap": None,  # TODO?
                "DepthImg": None,  # TODO?
                "StopDepthImg": None,  # TODO?
                "Pointcloud": None,  # TODO?
                "StopPointcloud": None,  # TODO?
                "PointcloudImg": None,  # TODO?
                "StopPointcloudImg": None,  # TODO?
            }
        }

        # IDX Remappings: Not necessary since we have a separate mechanism for genicam parameter assignment

        self.img_lock = threading.Lock()
        self.color_image_acquisition_running = False
        self.bw_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None

        rospy.loginfo(f"{self.node_name}: Launching NEPI IDX (ROS) interface...")
        self.idx_if = ROSIDXSensorIF(sensor_name=self.node_name,
                setResolutionModeCb=idx_callback_names["Controls"]["Resolution"],
                setFramerateModeCb=idx_callback_names["Controls"]["Framerate"],
                setContrastCb=idx_callback_names["Controls"]["Contrast"],
                setBrightnessCb=idx_callback_names["Controls"]["Brightness"],
                setThresholdingCb=idx_callback_names["Controls"]["Thresholding"],
                setRangeCb=idx_callback_names["Controls"]["Range"],
                getColor2DImgCb=idx_callback_names["Data"]["Color2DImg"],
                stopColor2DImgAcquisitionCb=idx_callback_names["Data"]["StopColor2DImg"],
                getGrayscale2DImgCb=idx_callback_names["Data"]["BW2DImg"],
                stopGrayscale2DImgAcquisitionCb=idx_callback_names["Data"]["StopBW2DImg"],
                getDepthMapCb=idx_callback_names["Data"]["DepthMap"],
                stopDepthMapAcquisitionCb=idx_callback_names["Data"]["StopDepthMap"],
                getDepthImgCb=idx_callback_names["Data"]["DepthImg"],
                stopDepthImgAcquisitionCb=idx_callback_names["Data"]["StopDepthImg"],
                getPointcloudCb=idx_callback_names["Data"]["Pointcloud"],
                stopPointcloudAcquisitionCb=idx_callback_names["Data"]["StopPointcloud"],
                getPointcloudImgCb=idx_callback_names["Data"]["PointcloudImg"],
                stopPointcloudImgAcquisitionCb=idx_callback_names["Data"]["StopPointcloudImg"])
        rospy.loginfo(f"{self.node_name}: ... IDX interface running")

        self.logDeviceInfo()

        self.idx_if.updateFromParamServer()

        rospy.spin()

    def logDeviceInfo(self):
        device_info_str = f"{self.node_name} info:\n"\
                + f"\tModel: {self.driver.model}\n"\
                + f"\tS/N: {self.driver.serial_number}\n"

        scalable_cam_controls = self.driver.getAvailableScaledCameraControls()
        discrete_cam_controls = self.driver.getAvailableDiscreteCameraControls()
        device_info_str += "\tCamera Controls:\n"
        for ctl in scalable_cam_controls:
            device_info_str += f"\t\t{ctl}\n"
        for ctl in discrete_cam_controls:
            device_info_str += f"\t\t{ctl}: {discrete_cam_controls[ctl]}\n"

        _, fmt = self.driver.getCurrentFormat()
        device_info_str += f"\tCamera Output Format: {fmt}\n"

        _, res_dict = self.driver.getCurrentResolution()
        device_info_str += "\tCurrent Resolution: " + f'{res_dict["width"]}x{res_dict["height"]}' + "\n"

        if (self.driver.hasAdjustableResolution()):
            _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
            device_info_str += "\tAvailable Resolutions:\n"
            for res in available_resolutions:
                device_info_str += "\t\t" + f'{res["width"]}x{res["height"]}' + "\n"

        if (self.driver.hasAdjustableFramerate()):
            _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()
            device_info_str += "\t" + f'Available Framerates (current resolution): {available_framerates}' + "\n"

        device_info_str += "\tResolution Modes:\n"
        for mode in self.resolution_mode_map:
            device_info_str += "\t\t"\
                    + f'{mode}: {self.resolution_mode_map[mode]["width"]}x{self.resolution_mode_map[mode]["height"]}'\
                    + "\n"

        device_info_str += "\tFramerate Modes (current resolution):\n"
        for mode in self.framerate_mode_map:
            device_info_str += f"\t\t{mode}: {self.framerate_mode_map[mode]}\n"

        rospy.loginfo(device_info_str)

    def createResolutionModeMapping(self):
        _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
        available_resolution_count = len(available_resolutions)
        # Check if this camera supports resolution adjustment
        if (available_resolution_count == 0):
            self.resolution_mode_map = {}
            return

        #available_resolutions is a list of dicts, sorted by "width" from smallest to largest
        # Distribute the modes evenly
        resolution_mode_count = ROSIDXSensorIF.RESOLUTION_MODE_MAX + 1
        # Ensure the highest resolution is available as "Ultra", others are spread evenly amongst remaining options
        self.resolution_mode_map = {resolution_mode_count - 1:available_resolutions[available_resolution_count - 1]}

        resolution_step = int(math.floor(available_resolution_count / resolution_mode_count))
        if resolution_step == 0:
            resolution_step = 1

        for i in range(1,resolution_mode_count):
            res_index = (available_resolution_count - 1) - (i*resolution_step)
            if res_index < 0:
                res_index = 0
            self.resolution_mode_map[resolution_mode_count - i - 1] = available_resolutions[res_index]

    def createFramerateModeMapping(self):
        _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()

        available_framerate_count = len(available_framerates)
        if (available_framerate_count == 0):
            self.framerate_mode_map = {}
            return

        #rospy.loginfo("Debug: Creating Framerate Mode Mapping")

        framerate_mode_count = ROSIDXSensorIF.FRAMERATE_MODE_MAX + 1
        # Ensure the highest framerate is available as "Ultra", others are spread evenly amongst remaining options
        self.framerate_mode_map = {framerate_mode_count - 1: available_framerates[available_framerate_count - 1]}

        framerate_step = int(math.floor(available_framerate_count / framerate_mode_count))
        if framerate_step == 0:
            framerate_step = 1

        for i in range(1, framerate_mode_count):
            framerate_index = (available_framerate_count - 1) - (i*framerate_step)
            if framerate_index < 0:
                framerate_index = 0
            self.framerate_mode_map[framerate_mode_count - i - 1] = available_framerates[framerate_index]

    def setResolutionMode(self, mode):
        if (mode >= len(self.resolution_mode_map)):
            return False, "Invalid mode value"

        resolution_dict = self.resolution_mode_map[mode]
        rospy.loginfo(self.node_name + ": setting driver resolution to " + str(resolution_dict['width']) + 'x' + str(resolution_dict['height']))
        status, msg = self.driver.setResolution(resolution_dict)
        if status is not False:
            # Need to update the framerate mode mapping in accordance with new resolution
            # And also restore the current framerate "mode" after the mapping
            current_framerate_mode = rospy.get_param('~framerate_mode', ROSIDXSensorIF.RESOLUTION_MODE_MAX)
            self.createFramerateModeMapping()
            self.setFramerateMode(current_framerate_mode)

        return status, msg
    
    def setFramerateMode(self, mode):
        if (mode >= len(self.framerate_mode_map)):
            return False, "Invalid mode value"

        fps = self.framerate_mode_map[mode]
        rospy.loginfo(self.node_name + ": setting driver framerate to " + str(fps))
        return self.driver.setFramerate(self.framerate_mode_map[mode])
    
    def setRatioParameter(self, param_name, ratio):
        if (ratio < 0.0) or (ratio > 1.0):
            return False, f"Invalid {param_name} ratio {ratio}"
        
        rospy.loginfo(self.node_name + ": Setting %s ratio to %0.2f", param_name, ratio)
        return self.driver.setScaledControl(param_name, ratio)
    
    def setContrastRatio(self, contrast_ratio):
        return self.setRatioParameter("contrast", contrast_ratio)
    
    def setBrightnessRatio(self, brightness_ratio):
        return self.setRatioParameter("brightness", brightness_ratio)
    
    def setThresholdingRatio(self, thresholding_ratio):
        return self.setRatioParameter("thresholding", thresholding_ratio)     

    def getColorImg(self):
        self.img_lock.acquire()
        # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        ret, msg = self.driver.startImageAcquisition()
        if ret is False:
            self.img_lock.release()
            return ret, msg, None, None

        self.color_image_acquisition_running = True

        timestamp = None

        start = time.time()
        frame, timestamp, ret, msg = self.driver.getImage()
        stop = time.time()
        #print('GI: ', stop - start)
        if ret is False:
            self.img_lock.release()
            return ret, msg, None, None

        if timestamp is not None:
            ros_timestamp = rospy.Time.from_sec(timestamp)
        else:
            ros_timestamp = rospy.Time.now()

        # Make a copy for the bw thread to use rather than grabbing a new frame
        if self.bw_image_acquisition_running:
            self.cached_2d_color_frame = frame
            self.cached_2d_color_frame_timestamp = ros_timestamp

        self.img_lock.release()
        return ret, msg, frame, ros_timestamp
    
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        if self.bw_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition()
        else:
            ret = True
            msg = "Success"
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_lock.release()
        return ret,msg
    
    def getBWImg(self):
        pass # TODO
    
    def stopBWImg(self):
        pass # TODO

        
if __name__ == '__main__':
    node = GenicamCameraNode()
