from ultralytics import YOLO
from ultralytics.yolo.v8.detect import DetectionPredictor
import pyrealsense2 as rs
import numpy as np
import cv2

model = YOLO("best.pt")

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # if not depth_frame or not color_frame:
        #     continue


        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        detections = model.predict(color_image)

        for detection in detections:
            print(detection)

        # cv2.imshow("Color", color_image)
        # #cv2.imshow("Depth", depth_image)
        # key = cv2.waitKey(2)
        # if key & 0xFF == ord("q"):
        #     cv2.destroyAllWindows()
        #     break

finally:
    pipeline.stop()





