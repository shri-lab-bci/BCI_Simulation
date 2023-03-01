#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import numpy as np

import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Twist

from sc_interaction.msg import ObjectInfo, ObjectInfos

class ObjectFilter(object):
    """ObjectFilter 클래스.
    위 클래스의 기능으로는
    1) 
    2)
    """
    def __init__(self):
        # 글로벌 변수 설정
        self.marker_info = ObjectInfos()
        self.object_info = ObjectInfos()

        self.nearest_object = ObjectInfo()
        self.fequency_list = np.zeros((20, 1), dtype=int)

        self.robot_pose = PoseWithCovarianceStamped()
        
        # 구독 설정
        marker_info_sub = Subscriber('recognition/marker3d_infos_tf', ObjectInfos) 
        object_info_sub = Subscriber('recognition/object_infos_tf', ObjectInfos)

        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_robot_pose)

        time_sync = ApproximateTimeSynchronizer([marker_info_sub, object_info_sub], queue_size=10, slop=0.5)
        time_sync.registerCallback(self.callback)

        # 발행 설정
        self.nearest_object_pub = rospy.Publisher("recognition/nearest_object", ObjectInfo, queue_size=1)

    def callback(self, markers, objects):
        marker_infos = markers
        object_infos = objects

        if len(marker_infos.object_infos) == 0 and len(object_infos.object_infos) == 0:
            self.nearest_object = ObjectInfo()
            self.nearest_object.id = 777

        elif len(marker_infos.object_infos) != 0 and len(object_infos.object_infos) == 0:
            distances = []
            for i in range(len(marker_infos.object_infos)):
                point_x = marker_infos.object_infos[i].point.x
                point_y = marker_infos.object_infos[i].point.y
                point = [float(point_x), float(point_y)]

                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                robot_point = [float(robot_x), float(robot_y)]

                distance = self.calculate_2d_distance(point, robot_point)
                distances.append(distance)

            min_distance = min(distances)
            min_index = distances.index(min_distance)

            nearest_marker = ObjectInfo()
            nearest_marker.id = marker_infos.object_infos[min_index].id
            nearest_marker.point.x = marker_infos.object_infos[min_index].point.x
            nearest_marker.point.y = marker_infos.object_infos[min_index].point.y
            self.nearest_object = nearest_marker

        elif len(marker_infos.object_infos) == 0 and len(object_infos.object_infos) != 0:
            distances = []
            for i in range(len(object_infos.object_infos)):
                point_x = object_infos.object_infos[i].point.x
                point_y = object_infos.object_infos[i].point.y
                point = [float(point_x), float(point_y)]

                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                robot_point = [float(robot_x), float(robot_y)]

                distance = self.calculate_2d_distance(point, robot_point)
                distances.append(distance)

            min_distance = min(distances)
            min_index = distances.index(min_distance)

            nearest_object = ObjectInfo()
            nearest_object.id = object_infos.object_infos[min_index].id
            nearest_object.point.x = object_infos.object_infos[min_index].point.x
            nearest_object.point.y = object_infos.object_infos[min_index].point.y
            self.nearest_object = nearest_object

        else:
            marker_distances = []
            for i in range(len(marker_infos.object_infos)):
                point_x = marker_infos.object_infos[i].point.x
                point_y = marker_infos.object_infos[i].point.y
                point = [float(point_x), float(point_y)]

                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                robot_point = [float(robot_x), float(robot_y)]

                distance = self.calculate_2d_distance(point, robot_point)
                marker_distances.append(distance)

            min_marker_distance = min(marker_distances)
            min_marker_index = marker_distances.index(min_marker_distance)

            object_distances = []
            for i in range(len(object_infos.object_infos)):
                point_x = object_infos.object_infos[i].point.x
                point_y = object_infos.object_infos[i].point.y
                point = [float(point_x), float(point_y)]

                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                robot_point = [float(robot_x), float(robot_y)]

                distance = self.calculate_2d_distance(point, robot_point)
                distances.append(distance)

            min_object_distance = min(object_distances)
            min_object_index = object_distances.index(min_object_distance)

            if min_object_distance <= min_marker_distance:
                nearest_object = ObjectInfo()
                nearest_object.id = object_infos.object_infos[min_object_index].id
                nearest_object.point.x = object_infos.object_infos[min_object_index].point.x
                nearest_object.point.y = object_infos.object_infos[min_object_index].point.y
                self.nearest_object = nearest_object
            else:
                nearest_marker = ObjectInfo()
                nearest_marker.id = marker_infos.object_infos[min_object_index].id
                nearest_marker.point.x = marker_infos.object_infos[min_object_index].point.x
                nearest_marker.point.y = marker_infos.object_infos[min_object_index].point.y
                self.nearest_object = nearest_marker

        try:
            self.nearest_object_pub.publish(self.nearest_object)

    def update_robot_pose(self, data):
        self.robot_pose = data.pose.pose

    def calculate_2d_distance(self, point1, point2):
        dist = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

        return dist

if __name__ == '__main__':
    print("Initializing Object-Filter node")
    rospy.init_node('object_filter', anonymous=False)
    object_filter = ObjectFilter()
    rospy.spin()
