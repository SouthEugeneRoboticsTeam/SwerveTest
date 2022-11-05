#!/usr/bin/env python3

import json
import time
import math

from cscore import CameraServer, VideoSource, UsbCamera, CvSink
from networktables import NetworkTablesInstance
from pupil_apriltags import Detector
import cv2
import numpy as np

server = CameraServer.getInstance()

ntinst = NetworkTablesInstance.getDefault()
ntinst.startClientTeam(2521)
ntinst.startDSClient()

width = 640
height = 480

cam = UsbCamera("Main", 0)
cam.setConnectionStrategy(UsbCamera.ConnectionStrategy.kKeepOpen)
cam.setResolution(width, height)

input_stream = CvSink("Input")
input_stream.setSource(cam)

# Table for vision output information
vision_table = ntinst.getTable('Vision')

cam_params = [685.55496655, 696.9549483, 329.08110703, 191.24604484]
detector = Detector(families='tag36h11')

# Wait for NetworkTables to start
time.sleep(0.5)

# try is for just in case
try:
    # Some use tag ids and also maybe send multiple instead of closest
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame(None)

        if frame_time == 0:
            continue

        detections = detector.detect(cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True,
                                         camera_params=cam_params, tag_size=0.15)

        vision_table.putBoolean('Is Target', len(detections) > 0)

        is_detection = len(detections) > 0
        vision_table.putBoolean('Is Target', is_detection)
        if is_detection:
            closest = max(detections,
                          key=lambda detection: detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[
                              2] ** 2)

            vision_table.putNumberArray('Position', [value[0] for value in closest.pose_t])
            vision_table.putNumberArray('Rotation', [value for row in closest.pose_R for value in row])

            radius = int(math.sqrt((closest.corners[0][0] - closest.corners[1][0]) ** 2 + (
                        closest.corners[0][1] - closest.corners[1][1]) ** 2) / 2)
            cv2.circle(input_img, (int(closest.center[0]), int(closest.center[1])), radius, (0, 0, 255), 1)

            cv2.polylines(input_img, np.array([detection.corners for detection in detections], dtype=np.int32), True,
                          (0, 255, 0))
            for detection in detections:
                dis = math.sqrt(detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[2] ** 2)
                cv2.putText(input_img, '{:.1E}'.format(dis), (int(detection.center[0]), int(detection.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0))

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        cv2.putText(input_img, str(round(fps, 1)), (0, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0))

except Exception as e:
    vision_table.putBoolean('Is Target', False)
    print(e)
