#!/usr/bin/env python


import sys
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from ultralytics import YOLO

rtsp_url = "rtsp://localhost:8554/stream"

def main():
  global rtsp_url
  rospy.init_node('stream_detect', anonymous=True)
  image_pub = rospy.Publisher("image_detected", Image, queue_size=10)
  bridge = CvBridge()

  # Set up the YOLOv8 model
  # model = YOLO('best.pt')

  # Set up the RealSense pipeline
  pipeline = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
  pipeline.start(config)

  image = Image
  r = rospy.Rate(100)
  while not rospy.is_shutdown():
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    out = cv2.VideoWriter(rtsp_url, fourcc, 25, (640, 480))
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
      continue
    frame = np.asanyarray(color_frame.get_data())
    # detections = model.predict(frame)
    # for detection in detections:
    #     out.write(frame)
    # print(frame.shape)

    image = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
    image_pub.publish(image)
    r.sleep()

  pipeline.stop()
  out.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()