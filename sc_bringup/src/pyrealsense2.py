#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np
import pyrealsense2 as rs

import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class RealsenseCamera(object):
    """Realsense 카메라를 동작한다."""

    def __init__(self):
        # 글로벌 변수 설정
        self.bridge = CvBridge()

        # 발행 설정
        # self.color_image_pub = rospy.Publisher("camera/color/image_raw", Image, queue_size=1)
        # self.colorful_depth_image_pub = rospy.Publisher("camera/colorful_depth/image_raw", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher("camera/depth/image_raw", Image, queue_size=1)
        self.compressed_color_image_pub = rospy.Publisher("camera/color/image_raw/compressed", CompressedImage, queue_size=1)

        # 디바이스 확인
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            print("연결된 카메라의 정보는 다음과 같습니다.")
            print("Device : {}".format(dev.get_info(rs.camera_info.name)))
            print("S/N : {}".format(dev.get_info(rs.camera_info.serial_number)))

        # 카메라 설정
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(rs_config)

        # Getting the depth sensor's depth scale (see rs-align example fr explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # 실행
        self.start_camera()

    def start_camera(self):
        """
        """
        while not rospy.is_shutdown():
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()

            aligned_frames = self.align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Intrinsics
            aligned_depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            # print("Aligned depth intrinsics : {}".format(type(aligned_depth_intrin)))
            # print("Aligned depth intrinsics coeffs : {}".format(aligned_depth_intrin.coeffs))
            # print("Aligned depth intrinsics fx : {}".format(aligned_depth_intrin.fx))
            # print("Aligned depth intrinsics fy : {}".format(aligned_depth_intrin.fy))
            # print("Aligned depth intrinsics height : {}".format(aligned_depth_intrin.height))
            # print("Aligned depth intrinsics width : {}".format(aligned_depth_intrin.width))
            # print("Aligned depth intrinsics model : {}".format(aligned_depth_intrin.model))
            # print("Aligned depth intrinsics ppx : {}".format(aligned_depth_intrin.ppx))
            # print("Aligned depth intrinsics ppy : {}".format(aligned_depth_intrin.ppy))

        
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            compressed_color_image = CompressedImage()
            compressed_color_image.header.stamp = rospy.Time.now()
            compressed_color_image.format = "jpeg"
            compressed_color_image.data = cv2.imencode('.jpg', color_image)[1].tostring()

            try:
                # self.color_image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
                # self.colorful_depth_image_pub.publish(self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8"))
                self.depth_image_pub.publish(self.bridge.cv2_to_imgmsg(depth_image, "16UC1"))
                self.compressed_color_image_pub.publish(compressed_color_image)

            except CvBridgeError as e:
                print(e)

if __name__ == '__main__':
    rospy.init_node('pyrealsense2', anonymous=False)
    realsense_camera = RealsenseCamera()
    rospy.spin()
