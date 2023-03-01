#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np
import pyrealsense2 as rs

import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from sc_interaction.msg import BoundingBoxes, ObjectCount, ObjectInfo, ObjectInfos

class RawInfoPublisher(object):
    """YOLO와 마커 인식 결과를 발행한다."""

    def __init__(self):
        # 글로벌 변수 설정
        self.bridge = CvBridge()
        self.recognition_image = np.zeros(shape=(480, 640))
        self.depth_image = np.zeros(shape=(480, 640))

        self.bounding_boxes = BoundingBoxes()

        # 카메라 특성값 설정
        self.aligned_depth_intrin = rs.intrinsics()
        self.aligned_depth_intrin.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.aligned_depth_intrin.fx = 617.44921875
        self.aligned_depth_intrin.fy = 617.110168457
        self.aligned_depth_intrin.height = 480
        self.aligned_depth_intrin.width = 640
        self.aligned_depth_intrin.model = rs.distortion.inverse_brown_conrady
        self.aligned_depth_intrin.ppx = 328.007507324
        self.aligned_depth_intrin.ppy = 241.498748779

        # 구독 설정
        rospy.Subscriber("camera/color/image_raw/compressed", CompressedImage, self.bridge_color_image)
        rospy.Subscriber('camera/depth/image_raw', Image, self.update_depth_image)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_boxes)
        rospy.Subscriber('darknet_ros/found_object', ObjectCount, self.update_object_count)
        rospy.Subscriber('recognition/marker2d_infos', Marker2DInfos, self.update_marker2d_infos)
        
        # 발행 설정
        self.pedestrian_info_pub = rospy.Publisher("recognition/pedestrian_infos", PedestrianInfos, queue_size=1)
        self.marker3d_info_pub = rospy.Publisher("recognition/marker3d_infos", Marker3DInfos, queue_size=1)
        self.marker_pub = rospy.Publisher("recognition/pedestrian_markers", MarkerArray, queue_size=1)
        self.recognition_image_pub = rospy.Publisher("recognition/image_raw/compressed", CompressedImage, queue_size=1)

    def bridge_color_image(self, data):
        """
        """

        # 압축 데이터를 CV 배열로 변환
        np_arr = np.fromstring(data.data, np.uint8)
        self.recognition_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def update_depth_image(self, data):
        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)
        
        self.depth_image = cv_depth_image

    def update_bounding_boxes(self, data):
        self.bounding_boxes = data

    def update_marker2d_infos(self, data):
        marker2d_infos = data

        marker3d_infos = Marker3DInfos()
        marker3d_markers = MarkerArray()

        if len(marker2d_infos.marker2d_infos) != 0:
            for i in range(len(marker2d_infos.marker2d_infos)):
                marker_id = marker2d_infos.marker2d_infos[i].id
                marker_x = int(marker2d_infos.marker2d_infos[i].point.x)
                marker_y = int(marker2d_infos.marker2d_infos[i].point.y)

                depth = self.depth_image[marker_y, marker_x] * 0.001

                depth_point = rs.rs2_deproject_pixel_to_point(self.aligned_depth_intrin, [marker_x, marker_y], float(depth))

                marker3d_info = Marker3DInfo()
                marker3d_info.id = int(marker_id)
                marker3d_info.point.x = float(depth_point[2])
                marker3d_info.point.y = float(depth_point[0]) * -1
                marker3d_info.point.z = float(depth_point[1])
                marker3d_infos.marker3d_infos.append(marker3d_info)
        else:
            marker3d_infos.marker3d_infos = []

        try:
            marker3d_infos.header = self.get_header()
            self.marker3d_info_pub.publish(marker3d_infos)
        except:
            pass

    def update_object_count(self, data):
        object_count = data.count
        bounding_boxes = self.bounding_boxes

        pedestrain_infos = PedestrianInfos()
        pedestrain_markers = MarkerArray()

        recognition_image = self.recognition_image.copy()

        if object_count != 0:
            try:
                for i in range(len(bounding_boxes.bounding_boxes)):
                    if bounding_boxes.bounding_boxes[i].Class == 'person':
                        probability = bounding_boxes.bounding_boxes[i].probability
                        xmin = bounding_boxes.bounding_boxes[i].xmin
                        ymin = bounding_boxes.bounding_boxes[i].ymin
                        xmax = bounding_boxes.bounding_boxes[i].xmax
                        ymax = bounding_boxes.bounding_boxes[i].ymax
                        _id = i + 1
                        _class = bounding_boxes.bounding_boxes[i].Class

                        depth_array = self.depth_image[ymin:ymax, xmin:xmax]
                        median_depth = np.median(depth_array)
                        median_depth_index = np.where(depth_array == median_depth)
                        for j in range(np.shape(median_depth_index)[1]):
                            cv2.circle(recognition_image, ((median_depth_index[1][j] + xmin), (median_depth_index[0][j] + ymin)), 7, (255, 0, 0), -1)

                        median_depth = median_depth * 0.001
                        print("median_depth : {}".format(median_depth))

                        center_ppx = int((xmin + xmax) / 2)
                        center_ppy = int((ymin + ymax) / 2)
                        depth_point = rs.rs2_deproject_pixel_to_point(self.aligned_depth_intrin, [center_ppx, center_ppy], median_depth)
                        # print("depth_point : {}".format(depth_point))

                        # Visualization
                        cv2.rectangle(recognition_image, (xmin, ymin), (xmax, ymax), (38, 199, 255), 2)
                        cv2.rectangle(recognition_image, (xmin - 1, ymin - 10), (xmax + 1, ymin + 10), (38, 199, 255), -1)
                        lineType = cv2.LINE_AA if cv2.__version__ > '3' else cv2.CV_AA
                        cv2.putText(recognition_image, '(' + str(i+1) + ') ' + _class + ' : %.2f' % probability, (xmin + 5, ymin + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, lineType)

                        pedestrain_info = PedestrianInfo()
                        pedestrain_info.id = int(_id)
                        pedestrain_info.point.x = float(depth_point[2])
                        pedestrain_info.point.y = float(depth_point[0]) * -1
                        pedestrain_info.point.z = float(depth_point[1])
                        pedestrain_infos.pedestrian_infos.append(pedestrain_info)

            except:
                # print("Bunding Box update error")
                pass

        else:
            pedestrain_infos.pedestrian_infos = []
            pedestrain_markers.markers = []

        try:
            pedestrain_infos.header = self.get_header()
            self.pedestrian_info_pub.publish(pedestrain_infos)

            compressed_recognition_image = CompressedImage()
            compressed_recognition_image.header.stamp = rospy.Time.now()
            compressed_recognition_image.format = "jpeg"
            compressed_recognition_image.data = cv2.imencode('.jpg', recognition_image)[1].tostring()
            self.recognition_image_pub.publish(compressed_recognition_image)

        except CvBridgeError as e:
            print(e)

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

if __name__ == '__main__':
    rospy.init_node('raw_info_publisher', anonymous=False)
    pedestrian_tracker = RawInfoPublisher()
    rospy.spin()
