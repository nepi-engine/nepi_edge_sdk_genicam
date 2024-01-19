#!/usr/bin/env python3
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
import cv2
from copy import deepcopy
import time
import threading

import numpy as np
from harvesters.core import Harvester
import genicam.genapi
from genicam.genapi import EAccessMode

# FIXME: incurring a lot of extra function calls when DBG is False, but handy for now
DBG=False
DBG_ROS=False
if DBG_ROS:
    import rospy
def DBG_PRINT(*args, **kwargs):
    if not DBG:
        return
    print_func = rospy.logfatal if DBG_ROS else print
    print_func(*args, **kwargs)

GENICAM_GENERIC_DRIVER_ID = 'GenericGenicam'

DEFAULT_GENTL_PATHS = [
        "/home/nepi/Downloads/Baumer_GAPI_SDK_2.14.1_lin_aarch64_cpp/lib/libbgapi2_gige.cti.2.14.1",
        "/home/nepi/Downloads/Baumer_GAPI_SDK_2.14.1_lin_aarch64_cpp/lib/libbgapi2_usb.cti.2.14.1",
        #"/opt/spinnaker/lib/flir-gentl/FLIR_GenTL.cti",
        #"/opt/ImpactAcquire/lib/arm64/mvGenTLProducer.cti",
]
DEFAULT_CAPTURE_TIMEOUT_S = 1

class GenicamCamDriver(object):
    MAX_CONSEC_FRAME_FAIL_COUNT = 3
    NODE_TYPE_MAP = {
            genicam.genapi.IBoolean: "bool",
            genicam.genapi.ICategory: "category",
            genicam.genapi.ICommand: "command",
            genicam.genapi.IEnumeration: "enum",
            genicam.genapi.IFloat: "float",
            genicam.genapi.IInteger: "int",
            genicam.genapi.IRegister: "register",
            genicam.genapi.IString: "string",
    }

    def __init__(self, model=None, serial_number=None, harvester=None, gentl_paths=DEFAULT_GENTL_PATHS):
        self.model = model
        self.serial_number = serial_number
        self.img_acq_running = False
        self.img_acq_lock = threading.Lock()
        self.consec_failed_frames = 0
        self.connected = False
        self.resolution = None
        DBG_PRINT(f"Initializing {self._name()}")

        # Make sure harvester library handle is up to date.
        self.harvester = harvester
        if self.harvester is None:
            self.harvester = Harvester()
            if isinstance(gentl_paths, str):
                gentl_paths = [gentl_paths]
            for path in gentl_paths:
                self.harvester.add_file(path)
            DBG_PRINT("Detecting devices...")
            self.harvester.update()
        if len(self.harvester.device_info_list) < 1:
            raise Exception("No genicam devices detected")
        DBG_PRINT(f"{len(self.harvester.device_info_list)} genicam devices detected")

        # Find and connect to the specified device.
        search_keys = dict()
        search_keys["access_status"] = genicam.gentl.DEVICE_ACCESS_STATUS_LIST.DEVICE_ACCESS_STATUS_READWRITE.value
        if self.model is not None:
            search_keys["model"] = self.model
        if self.serial_number is not None:
            search_keys["serial_number"] = self.serial_number
        try:
            self.device = self.harvester.create(search_keys)
        except ValueError as err:
            raise
            raise Exception("Failed to find or uniquely identify camera")
        self.model = self.device.remote_device.module.model
        self.serial_number = self.device.remote_device.module.serial_number
        self.connected = True
        DBG_PRINT(f"Connected to {self._name()}")
        DBG_PRINT("Discovering camera controls")

        # Enable frame rate adjustment, if present.
        try:
            self.device.remote_device.node_map.\
                    __getattr__("AcquisitionFrameRateEnable").value = True
            DBG_PRINT("Set AcquisitionFrameRateEnable to True")
        except AttributeError:
            DBG_PRINT("No AcquisitionFrameRateEnable node")

        # Enable gamma adjustment, if present.
        try:
            self.device.remote_device.node_map.\
                    __getattr__("GammaEnable").value = True
            DBG_PRINT("Set GammaEnable to True")
        except AttributeError:
            DBG_PRINT("No GammaEnable node")

        ret, msg = self.initCameraControlsDict()
        if not ret:
            DBG_PRINT("Initialization failed, failed to initialize camera controls: " + msg)
        else:
            DBG_PRINT("Initialization complete")

    def _name(self):
        model = "unspecified" if self.model is None else self.model
        serial_number = "unspecified" if self.serial_number is None\
                else self.serial_number
        return f"model: {model}, serial number: {serial_number}"

    def _setNodeVal(self, name, val):
        name = name.lower()
        ret = True
        msg = "Success"

        self.img_acq_lock.acquire()
        img_acq_running_cache = self.img_acq_running
        if img_acq_running_cache:
            self.stopImageAcquisition(hold_lock=True)

        try:
            DBG_PRINT(f"Attempting to set node {name} to {val}...")
            self.camera_controls[name]["node"].value = val
        except KeyError:
            ret, msg = False, f"Node does not exist (set {name} {val})"
        except genicam.genapi.AccessException:
            ret, msg = False, f"Permission denied (set {name} {val})"
        except Exception as e:
            ret, msg = False, f"Error: {e} (set {name} {val})"

        if ret:
            # A bit convoluted. If our param is not a float, then we see if the
            # readback value equals the intended value. If not, it's a readback
            # error. Comparing floats for equality is bad, so, if it's a float
            # we just check to see if it's close. If it's not close, then it's a
            # readback error.
            rb, msg = self._getNodeVal(name)
            pct_delta = abs(rb - val) / val
            MAX_ALLOWABLE_READBACK_DELTA_FLOAT = 0.01  # 1%
            rb_error = False
            if isinstance(val, float):
                rb_error = rb_error or pct_delta > MAX_ALLOWABLE_READBACK_DELTA_FLOAT
            else:
                rb_error = rb_error or (rb != val)
            if rb_error:
                ret, msg = False, f"Readback error: {rb} != {val} (set {name} {val})"

        if ret:
            DBG_PRINT(f"Successfully set node {name} to {val}")
        else:
            DBG_PRINT(f"Failed to set node {name} to {val}")

        if img_acq_running_cache:
            self.startImageAcquisition(hold_lock=True)
        self.img_acq_lock.release()

        return ret, msg

    def _getNodeVal(self, name):
        name = name.lower()
        ret = None
        try:
            ret = self.camera_controls[name]["node"].value
        except KeyError:
            return ret, f"Node does not exist (get {name})"
        except genicam.genapi.AccessException:
            return ret, f"Permission denied (get {name})"
        except Exception as e:
            return ret, f"Error: {e} (get {name})"
        return ret, "Success"

    def isConnected(self):
        return self.connected

    def initCameraControlsDict(self):
        if not self.isConnected():
            raise RuntimeError("Must connect before initializing camera controls")
        self.camera_controls = dict()
        node_names = [n for n in self.device.remote_device.node_map.__dir__() if n[0].isupper()]
        for node_name in node_names:
            node = self.device.remote_device.node_map.__getattr__(node_name)
            entry = dict()
            # NOTE: we keep a handle on the node rather than reading/storing the value.
            #       The value can be read/written via the node's .value member.
            entry["node"] = node
            access_mode = EAccessMode(node.get_access_mode())
            entry["readable"] = access_mode in (EAccessMode.RO, EAccessMode.RW)
            entry["writable"] = access_mode in (EAccessMode.WO, EAccessMode.RW)
            entry["type"] = self.NODE_TYPE_MAP[type(node)]
            if entry["readable"]:
                if entry["type"] == "bool":
                    pass # Nothing to do here but store it.
                elif entry["type"] == "category":
                    pass # These seem to just be a way of grouping other nodes.
                elif entry["type"] == "command":
                    pass # Nothing to do here but store it. 
                elif entry["type"] == "enum":
                    entry["options"] = list()
                    for option in node.symbolics:
                        entry["options"].append(option)
                    entry["value"] = node.value
                elif entry["type"] == "float":
                    entry["min"] = node.min
                    entry["max"] = node.max
                elif entry["type"] == "int":
                    entry["min"] = node.min
                    entry["max"] = node.max
                    entry["step"] = node.inc
                elif entry["type"] == "register":
                    pass # Can't tell if this is useful for anything.
                elif entry["type"] == "string":
                    pass # Nothing to do here but store it.
            self.camera_controls[node_name.lower()] = entry

        # Attempt to set initial resolution to whatever the current device
        # setting is. We need to do this because resolution is handled in the
        # driver, not in the device. If this fails we still return success,
        # because the driver is still usable even if we can't change the
        # resolution. In this case, either width or height will be set to
        # None, in which case self.hasAdjustableCameraControl will return
        # False, so callers will no not to try to set resolution.
        width, msg = self._getNodeVal("Width")
        height, msg = self._getNodeVal("Height")
        if width is None:
            DBG_PRINT(f"Failed setting initial width: {msg}")
        if height is None:
            DBG_PRINT(f"Failed setting initial height: {msg}")
        self.setResolution({"width": width, "height": height})

        return True, "Success"

    def initVideoFormatDict(self):
        pass

    def getAvailableScaledCameraControls(self):
        numeric_ctl_list = list()
        scaled_types = ("float", "int")
        for ctl in self.camera_controls:
            if not self.camera_controls[ctl]["readable"]:
                continue
            if not self.camera_controls[ctl]["writable"]:
                continue
            if self.camera_controls[ctl]["type"] in scaled_types:
                numeric_ctl_list.append(ctl)
        return numeric_ctl_list

    def getAvailableDiscreteCameraControls(self):
        switched_ctl_dict = dict()
        for ctl in self.camera_controls:
            if not self.camera_controls[ctl]["readable"]:
                continue
            if not self.camera_controls[ctl]["writable"]:
                continue
            typ = self.camera_controls[ctl]["type"]
            if typ == "bool":
                switched_ctl_dict[ctl] = ["off", "on"]
            elif typ == "enum":
                switched_ctl_dict[ctl] = self.camera_controls[ctl]["options"]
        return switched_ctl_dict

    def setScaledCameraControl(self, genicam_setting_name, scaled_val):
        if scaled_val < 0.0 or scaled_val > 1.0:
            return False, f"Invalid scaled value for {genicam_setting_name}"

        if genicam_setting_name not in self.camera_controls:
            return False, f"Unavailable setting: {genicam_setting_name}"

        vmax = self.camera_controls[genicam_setting_name]["max"]
        vmin = self.camera_controls[genicam_setting_name]["min"]
        diff = vmax - vmin
        raw_float = vmin + (diff * scaled_val)
        step = self.camera_controls[genicam_setting_name]["step"]
        raw_step = int(round(raw_float / step)) * step
        stepped_val = raw_step * step

        return self._setNodeVal(genicam_setting_name, stepped_val)

    def getScaledCameraControl(self, genicam_setting_name):
        if genicam_setting_name not in self.camera_controls:
            return False, f"Unavailable setting: {genicam_setting_name}"

        val, msg = self._getNodeVal(genicam_setting_name)

        if val is None:
            return False, f"Failed to get value from device: {msg}"

        vmin = self.camera_controls[genicam_setting_name]["min"]
        vmax = self.camera_controls[genicam_setting_name]["max"]
        diff = vmax - vmin
        ret = float(val - vmin) / float(diff)
        return ret, "Success"

    def hasAdjustableCameraControl(self, genicam_setting_name):
        if genicam_setting_name not in self.camera_controls:
            return False
        return self.camera_controls[genicam_setting_name]["writable"]

    def hasAdjustableResolution(self):
        if "height" not in self.camera_controls or "width" not in self.camera_controls:
            return False
        if self.resolution["width"] is None or self.resolution["height"] is None:
            return False
        return self.camera_controls["height"]["writable"] and \
                self.camera_controls["width"]["writable"]

    def hasAdjustableFramerate(self):
        if "acquisitionframerate" not in self.camera_controls:
            return False
        if not self.camera_controls["acquisitionframerate"]["writable"]:
            return False
        return True

    def getCurrentVideoSettings(self):
        video_settings_dict = dict()
        fmt, msg = self._getNodeVal("PixelFormat")
        if fmt is None:
            # If we fail to read the pixel format we still want to return
            # successfully, because we should still be able to provide width
            # and height. So, we just log the error and move on.
            DBG_PRINT(msg)
        width = self.resolution["width"]
        height = self.resolution["height"]
        video_settings_dict["format"] = fmt
        video_settings_dict["width"] = self.resolution["width"]
        video_settings_dict["height"] = self.resolution["height"]
        return True, video_settings_dict

    def getCurrentResolution(self):
        return True, self.resolution

    def setResolution(self, resolution_dict):
        DBG_PRINT(f"setResolution {resolution_dict}")
        width_too_small = resolution_dict["width"] < self.camera_controls["width"]["min"]
        width_too_large = resolution_dict["width"] > self.camera_controls["width"]["max"]
        height_too_small = resolution_dict["height"] < self.camera_controls["height"]["min"]
        height_too_large = resolution_dict["height"] > self.camera_controls["height"]["max"]
        if width_too_small or width_too_large or height_too_small or height_too_large:
            return False, "Requested resolution is not available"
        self.resolution = resolution_dict
        return True, "Success"

    def getCurrentResolutionAvailableFramerates(self):
        NUM_OPTIONS = 20
        available_framerates = [x for x in np.linspace(
            self.camera_controls["acquisitionframerate"]["min"],
            self.camera_controls["acquisitionframerate"]["max"],
            NUM_OPTIONS
        )]
        DBG_PRINT(available_framerates)
        return True, available_framerates

    def setFramerate(self, max_fps):
        if not self.hasAdjustableFramerate():
            return False, "Framerate is not adjustable"
        fps_too_low = max_fps < self.camera_controls["acquisitionframerate"]["min"]
        fps_too_high = max_fps > self.camera_controls["acquisitionframerate"]["max"]
        if fps_too_low or fps_too_high:
            return False, "Invalid framerate requested"
        return self._setNodeVal("AcquisitionFrameRate", max_fps)

    def getFramerate(self):
        if "acquisitionframerate" not in self.camera_controls:
            return False, "Camera does not provide framerate information"
        return self._getNodeVal("AcquisitionFrameRate")

    def getCurrentFormat(self):
        return self._getNodeVal("PixelFormat")

    def getCurrentFormatAvailableResolutions(self):
        # NOTE: clamp resolution between 240x320 to 1080x1920
        # FIXME: move this to config
        NUM_OPTIONS = 4
        min_height = max(self.camera_controls["height"]["min"], 240)
        min_width = max(self.camera_controls["width"]["min"], 320)
        max_height = min(self.camera_controls["height"]["max"], 1080)
        max_width = min(self.camera_controls["width"]["max"], 1920)
        height_step = self.camera_controls["height"]["step"]
        width_step = self.camera_controls["width"]["step"]
        heights = [int(x) for x in np.linspace(min_height, max_height, NUM_OPTIONS)]
        widths = [int(x) for x in np.linspace(min_width, max_width, NUM_OPTIONS)]
        heights[0] += (height_step - (heights[0] % height_step))
        widths[0] += (width_step - (widths[0] % width_step))
        for i in range(1, NUM_OPTIONS):
            heights[i] -= (heights[i] % height_step)
            widths[i] -= (widths[i] % width_step)
        return True, [{"height": h, "width": w} for (h, w) in zip(heights, widths)]

    def imageAcquisitionRunning(self):
        return self.img_acq_running

    def startImageAcquisition(self, hold_lock=False):
        if not self.connected:
            return False, "Not connected to device"
        if self.img_acq_running:
            return True, "Already capturing from genicam device " + self._name()

        if not hold_lock:
            self.img_acq_lock.acquire()
        self.img_acq_running = True
        DBG_PRINT("starting device...")
        self.device.start()
        DBG_PRINT("device started")
        if not hold_lock:
            self.img_acq_lock.release()
        return True, "Success"

    def stopImageAcquisition(self, hold_lock=False):
        if not self.img_acq_running:
            return True, "Not presently capturing from genicam device " + self._name()
        if not hold_lock:
            self.img_acq_lock.acquire()
        DBG_PRINT("stopping device...")
        self.device.stop()
        time.sleep(0.05)
        DBG_PRINT("device stopped")
        self.img_acq_running = False
        if not hold_lock:
            self.img_acq_lock.release()
        return True, "Success"

    def getImage(self, timeout_s=DEFAULT_CAPTURE_TIMEOUT_S):
        # Command the target device to capture an image.
        if not self.img_acq_running:
            self.startImageAcquisition()
        try:
            buf = self.device.fetch(timeout=timeout_s)
        except genicam.gentl.TimeoutException:
            self.consec_failed_frames += 1
            return None, None, False,\
                    f"Timed out fetching image from device ({self.consec_failed_frames})"
        except Exception as e:
            self.consec_failed_frames += 1
            return None, None, False, f"Error: {e} (getImage)"
        timestamp = time.time()

        # Grab the image itself from the buffer, copy it, and make it 2D.
        image = buf.payload.components[0]
        frame = deepcopy(image.data)
        frame.resize((image.height, image.width))

        # Reque the buffer so that it can be used again.
        buf.queue()

        # Convert to RGB and resize. Note that resolution is tracked by the
        # driver and does not necessarily use the values of the Height and
        # Width nodes. This is because those nodes select the range of pixels
        # to return from the sensor, not the pixel pitch.
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2RGB)
        if frame.shape != (self.resolution["width"], self.resolution["height"]):
            frame = cv2.resize(
                    src=frame,
                    dsize=(self.resolution["width"], self.resolution["height"]),
                    interpolation=cv2.INTER_AREA,
            )

        # Success. Reset fail counter.
        self.consec_failed_frames = 0
        return frame, timestamp, True, "Success"

def test_genicam_cam_driver(model, serial_number, harvester, test_camera_controls=True,
        test_image_acquisition=True, dump_lut=False):
    DBG_PRINT(f"\nTESTING {model}/{serial_number}\n")
    cam = GenicamCamDriver(model=model, serial_number=serial_number,
            harvester=harvester)
    if test_camera_controls:
        for c in cam.camera_controls:
            DBG_PRINT(c)
            DBG_PRINT(cam.camera_controls[c])
    last_frame = None
    if test_image_acquisition:
        cam.startImageAcquisition()
        num_frames = 60
        num_successful_frames = 0
        t0 = time.time()
        DBG_PRINT(f"Attempting to capture {num_frames} frames...")
        for _ in range(num_frames):
            frame, timestamp, ret, msg = cam.getImage()
            if ret:
                num_successful_frames += 1
                last_frame = frame
            else:
                DBG_PRINT("\nMissed frame: " + msg)
        if last_frame is not None:
            duration = time.time() - t0
            fps = num_successful_frames / duration
            DBG_PRINT(f"Captured {num_successful_frames} frames in {duration:0.3f} seconds ({fps} fps).")
            plt.rcParams["figure.figsize"] = (12, 8)
            plt.imshow(last_frame)
            plt.show()
        cam.stopImageAcquisition()
        cam.device.destroy()
    if dump_lut:
        cam.camera_controls["lutenable"]["node"].value = True
        self._setNodeVal("LUTEnable", True)
        imin = cam.camera_controls["lutindex"]["min"]
        imax = cam.camera_controls["lutindex"]["max"]
        print("Dumping contents of LUT...")
        for i in range(imin, imax + 1):
            cam.camera_controls["lutindex"] = i
            val = cam.camera_controls["lutvalue"]["node"].value
            print(f"{i}: {val}")
        print("Done.")

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    h = Harvester()
    for path in DEFAULT_GENTL_PATHS:
        h.add_file(path)
    h.update()
    devices_to_test = h.device_info_list
    DBG_PRINT(devices_to_test)
    for dut in devices_to_test:
        test_genicam_cam_driver(model=dut.model,
                serial_number=dut.serial_number,
                harvester=h,
                test_camera_controls=False,
                test_image_acquisition=True,
                dump_lut=False,
        )
