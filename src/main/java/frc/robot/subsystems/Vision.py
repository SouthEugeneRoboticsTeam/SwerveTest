#!/usr/bin/env python3

import json
import time
import sys
import math

from cscore import CameraServer, VideoSource, UsbCamera
from networktables import NetworkTables
from pupil_apriltags import Detector
import cv2
import numpy as np

server = CameraServer()

with open('/boot/frc.json') as f:
    config = json.load(f)
camera = config['cameras'][0]

width = camera['width']
height = camera['height']

server.startAutomaticCapture()

input_stream = server.getVideo()
output_stream = server.putVideo('Processed', width, height)

# Table for vision output information
vision_table = NetworkTables.getTable('vision')

cam_params = [699.86650947, 358.12604168, 699.46805447, 177.13035609]
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
            output_stream.notifyError(input_stream.getError())
            continue

        # 200 mm target for some reason tag_size seems to off by x10
        detections = detector.detect(cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True,
                                     camera_params=[1078.03779, 1084.50988, 580.850545, 245.959325], tag_size=0.02)

        vision_table.putBoolean('is_target', len(detections) > 0)

        is_detection = len(detections) > 0
        vision_table.putBoolean('is_target', is_detection)
        if is_detection:
            closest = max(detections,
                          key=lambda detection: detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[
                              2] ** 2)

            vision_table.putNumberArray('position', [value[0] for value in closest.pose_t])
            vision_table.putNumberArray('rotation', [value for row in closest.pose_R for value in row])

            radius = int(math.sqrt((closest.corners[0][0] - closest.corners[1][0]) ** 2 + (
                        closest.corners[0][1] - closest.corners[1][1]) ** 2) / 2)
            cv2.circle(input_img, (int(closest.center[0]), int(closest.center[1])), radius, (0, 0, 255), 1)

            cv2.polylines(input_img, np.array([detection.corners for detection in detections], dtype=np.int32), True,
                          (0, 255, 0))
            for detection in detections:
                dis = math.sqrt(detection.pose_t[0] ** 2 + detection.pose_t[1] ** 2 + detection.pose_t[2] ** 2)
                cv2.putText(input_img, '{:.1E}'.format(dis), (int(detection.center[0]), int(detection.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        cv2.putText(input_img, str(round(fps, 1)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

        output_stream.putFrame(input_img)

except Exception as e:
    vision_table.putBoolean('is_target', False)
    print(e)
