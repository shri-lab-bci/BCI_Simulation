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

class ObjectDeprojector(object):
    """Deprojector 클래스.
    위 클래스의 기능으로는
    1) YOLO와 마커 인식 결과(2D Pixel Point)를 실제 좌표값(3D Point)로 변환하여 발행한다.
    """
    def __init__(self):
        # 글로벌 변수 설정
        self.bridge = CvBridge()
        self.color_image = np.zeros(shape=(480, 640))
        self.depth_image = np.zeros(shape=(480, 640))

        self.markers = ObjectInfos()
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

        self.nearest_marker = ObjectInfo()
        self.nearest_object = ObjectInfo()

        # 
        self.nearest_object_info = ObjectInfo()

        # 구독 설정
        rospy.Subscriber("camera/color/image_raw/compressed", CompressedImage, self.bridge_color_image)
        rospy.Subscriber('camera/depth/image_raw', Image, self.update_depth_image)

        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_boxes)
        rospy.Subscriber('darknet_ros/found_object', ObjectCount, self.update_object_count)

        rospy.Subscriber('recognition/marker2d_infos', ObjectInfos, self.update_marker2d_infos)
        
        # 발행 설정
        self.object_info_pub = rospy.Publisher("recognition/object_infos", ObjectInfos, queue_size=1)
        self.marker3d_info_pub = rospy.Publisher("recognition/marker3d_infos", ObjectInfos, queue_size=1)
        self.recognition_image_pub = rospy.Publisher("recognition/image_raw/compressed", CompressedImage, queue_size=1)

    def bridge_color_image(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

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

        marker3d_infos = ObjectInfos()
        marker3d_markers = MarkerArray()

        if len(marker2d_infos.object_infos) != 0:
            for i in range(len(marker2d_infos.object_infos)):
                marker_id = marker2d_infos.object_infos[i].id
                marker_x = int(marker2d_infos.object_infos[i].point.x)
                marker_y = int(marker2d_infos.object_infos[i].point.y)

                depth = self.depth_image[marker_y, marker_x] * 0.001

                depth_point = rs.rs2_deproject_pixel_to_point(self.aligned_depth_intrin, [marker_x, marker_y], float(depth))

                # Visualization
                if depth >= 0.5 and depth <= 5.0:
                    circle_size = 12
                    circle_color = (255, 0, 0)
                else:
                    circle_size = 5
                    circle_color = (0, 255, 255)
                        
                cv2.circle(self.color_image, (marker_x, marker_y), circle_size, circle_color, -1)

                marker3d_info = ObjectInfo()
                marker3d_info.id = int(marker_id)
                marker3d_info.point.x = float(depth_point[2])
                marker3d_info.point.y = float(depth_point[0]) * -1
                marker3d_info.point.z = float(depth_point[1])
                marker3d_infos.object_infos.append(marker3d_info)

        else:
            marker3d_infos.object_infos = []

        try:
            marker3d_infos.header = self.get_header()
            self.marker3d_info_pub.publish(marker3d_infos)

        except:
            pass

    def update_object_count(self, data):
        object_count = data.count
        bounding_boxes = self.bounding_boxes

        object_infos = ObjectInfos()

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

                        median_depth = median_depth * 0.001

                        center_ppx = int((xmin + xmax) / 2)
                        center_ppy = int((ymin + ymax) / 2)
                        depth_point = rs.rs2_deproject_pixel_to_point(self.aligned_depth_intrin, [center_ppx, center_ppy], median_depth)

                        # Visualization
                        
                        if median_depth >= 0.5 and median_depth <= 5.0:
                            circle_size = 12
                            circle_color = (0, 0, 255)
                        else:
                            circle_size = 5
                            circle_color = (0, 255, 255)
                                
                        cv2.circle(self.color_image, (center_ppx, center_ppy), circle_size, circle_color, -1)
                        # cv2.rectangle(self.color_image, (xmin, ymin), (xmax, ymax), (38, 199, 255), 2)
                        # cv2.rectangle(self.color_image, (xmin - 1, ymin - 10), (xmax + 1, ymin + 10), (38, 199, 255), -1)
                        # lineType = cv2.LINE_AA if cv2.__version__ > '3' else cv2.CV_AA
                        # cv2.putText(self.color_image, '(' + str(i+1) + ') ' + _class + ' : %.2f' % probability, (xmin + 5, ymin + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, lineType)

                        object_info = ObjectInfo()
                        object_info.id = int(_id)
                        object_info.point.x = float(depth_point[2])
                        object_info.point.y = float(depth_point[0]) * -1
                        object_info.point.z = float(depth_point[1])
                        object_infos.object_infos.append(object_info)

            except:
                pass

        else:
            object_infos.object_infos = []

        try:
            object_infos.header = self.get_header()
            self.object_info_pub.publish(object_infos)

            compressed_recognition_image = CompressedImage()
            compressed_recognition_image.header.stamp = rospy.Time.now()
            compressed_recognition_image.format = "jpeg"
            compressed_recognition_image.data = cv2.imencode('.jpg', self.color_image)[1].tostring()
            self.recognition_image_pub.publish(compressed_recognition_image)

        except CvBridgeError as e:
            pass

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    def calculate_2d_distance(self, point):
        origin_point = [0, 0]
        dist = np.sqrt((point[0] - origin_point[0])**2 + (point[1] - origin_point[1])**2)

        return dist

if __name__ == '__main__':
    print("Initializing Object-Deprojector node")
    rospy.init_node('object_deprojector', anonymous=False)
    object_deprojector = ObjectDeprojector()
    rospy.spin()
