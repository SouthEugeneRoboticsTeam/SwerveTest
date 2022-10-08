#!/usr/bin/env python3

import json
import time
import sys
import math

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
from pupil_apriltags import Detector
import cv2
import numpy as np

configFile = "/boot/frc.json"


class CameraConfig:
    pass


team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(error):
    """Report parse error."""
    print("config error in '" + configFile + "': " + error, file=sys.stderr)


def read_camera_config(cam_config):
    cam = CameraConfig()

    # name
    try:
        cam.name = cam_config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = cam_config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = cam_config.get("stream")

    cam.config = cam_config

    cameraConfigs.append(cam)
    return True


def read_switched_camera_config(cam_config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = cam_config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = cam_config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True


def read_config():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        text = j["ntmode"]
        if text.lower() == "client":
            server = False
        elif text.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(text))

    # cameras
    try:
        cams = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for cam in cams:
        if not read_camera_config(cam):
            return False

    # switched cameras
    if "switched cameras" in j:
        for cam in j["switched cameras"]:
            if not read_switched_camera_config(cam):
                return False

    return True


def start_camera(cam_config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(cam_config.name, cam_config.path))
    inst = CameraServer.getInstance()
    usb_cam = UsbCamera(cam_config.name, cam_config.path)
    usb_server = inst.startAutomaticCapture(camera=usb_cam, return_server=True)

    usb_cam.setConfigJson(json.dumps(cam_config.config))
    usb_cam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if cam_config.streamConfig is not None:
        usb_server.setConfigJson(json.dumps(cam_config.streamConfig))

    return usb_cam


def start_switched_camera(cam_config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(cam_config.name, cam_config.key))
    cam_server = CameraServer.getInstance().addSwitchedCamera(cam_config.name)

    def listener(_, key, value, _):
        if isinstance(value, float):
            i = int(value)
            if 0 <= i < len(cameras):
                cam_server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    cam_server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(cam_config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return cam_server


if len(sys.argv) >= 2:
    configFile = sys.argv[1]

# read configuration
if not read_config():
    sys.exit(1)

# start NetworkTables
ntinst = NetworkTablesInstance.getDefault()
if server:
    print("Setting up NetworkTables server")
    ntinst.startServer()
else:
    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)
    ntinst.startDSClient()

# start cameras
for config in cameraConfigs:
    cameras.append(start_camera(config))

# start switched cameras
for config in switchedCameraConfigs:
    start_switched_camera(config)

server = CameraServer.getInstance()

camera = cameras[0]
detector = Detector(families='tag36h11')

width = 160
height = 120

input_stream = server.getVideo()
output_stream = server.putVideo('Processed', width, height)

# Table for vision output information
vision_table = ntinst.getTable('vision')

# Wait for NetworkTables to start
time.sleep(0.5)

# try is for just in case
try:
    # Some use tag ids and also maybe send multiple instead of closest
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame()

        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        detections = detector.detect(cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True, camera_params=[1078.03779, 1084.50988, 580.850545, 245.959325], tag_size=0.1)

        vision_table.putBoolean('is_target', len(detections) > 0)

        is_detection = len(detections) > 0
        vision_table.putBoolean('is_target', is_detection)
        if is_detection:
            closest = max(detections, key=lambda detection: detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[2] ** 2)

            vision_table.putNumberArray('position', [value[0] for value in closest.pose_t])
            vision_table.putNumberArray('rotation', [value for row in closest.pose_R for value in row])

            radius = int(math.sqrt((closest.corners[0][0] - closest.corners[1][0]) ** 2 + (closest.corners[0][1] - closest.corners[1][1]) ** 2) / 2)
            cv2.circle(input_img, (int(closest.center[0]), int(closest.center[1])), radius, (0, 0, 255), 1)

            cv2.polylines(input_img, np.array([detection.corners for detection in detections], dtype=np.int32), True, (0, 255, 0))
            for detection in detections:
                dis = math.sqrt(detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[2] ** 2)
                cv2.putText(input_img, '{:.1E}'.format(dis), (int(detection.center[0]), int(detection.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        cv2.putText(input_img, str(round(fps, 1)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

        output_stream.putFrame(input_img)

except Exception as e:
    vision_table.putBoolean('is_target', False)
    print(e)
