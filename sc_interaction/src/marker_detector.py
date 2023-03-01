#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np
from sc_interaction.msg import ObjectInfo, ObjectInfos

class MarkerDetector:
    """마커 인식기 클래스.
    위 클래스의 기능으로는
    1) 마커를 인식하며
    2) 인식된 마커의 아이디와 픽셀 포인트를 토픽으로 발행한다.
    """
    def __init__(self):
        # 글로벌 변수 설정
        self.bridge = CvBridge()
        self.image = None

        # 구독 설정
        self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback)

        # 발행 설정
        self.marker2d_info_pub = rospy.Publisher("recognition/marker2d_infos", ObjectInfos, queue_size=1)    

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        markers_img = self.detect_aruco(cv_image)

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        marker2d_infos = ObjectInfos()

        if len(corners) != 0:
            for i in range(len(corners)):
                center = np.mean(corners[i][0], axis=0)
                center_x, center_y = center[0], center[1]
                # cv2.circle(img, (center_x, center_y), 5, (0,0,255), -1)

                marker2d_info = ObjectInfo()
                marker2d_info.id = int(ids[i])
                marker2d_info.point.x = float(center_x)
                marker2d_info.point.y = float(center_y)
                marker2d_infos.object_infos.append(marker2d_info)

        try:
            marker2d_infos.header = self.get_header()
            self.marker2d_info_pub.publish(marker2d_infos)
            # output = aruco.drawDetectedMarkers(self.image, corners, ids)  # detect the sruco markers and display its aruco id.
            return output
        except:
            pass

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

def main():
    print("Initializing Marker-Detector node")
    rospy.init_node('marker_detector', anonymous=False)
    marker_detector = MarkerDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
