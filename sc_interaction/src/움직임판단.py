#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from pedestrian_tracking.msg import PedestrianInfo, PedestrianInfos

class ObjectInfoManager(object):
    """
    """

    def __init__(self):
        # 글로벌 변수 설정
        self.object_infos = np.zeros((10, 10), dtype=float)
        self.last_pose_x = 0
        self.last_pose_y = 0
        self.last_time = 0

        # 구독 설정
        rospy.Subscriber("/recognition/pedestrian_infos_tf", PedestrianInfos, self.update_info)
        
        # 발행 설정
        self.marker_pub = rospy.Publisher("recognition/pedestrian_markers", MarkerArray, queue_size=1)

    def update_info(self, data):
        """
        """
        pedestrian_infos = data

        pedestrian_markers = MarkerArray()

        if len(pedestrian_infos.pedestrian_infos) != 0:
            for _id in range(len(pedestrian_infos.pedestrian_infos)):
                object_id = int(pedestrian_infos.pedestrian_infos[_id].id)

                pos_x = pedestrian_infos.pedestrian_infos[_id].point.x
                pos_y = pedestrian_infos.pedestrian_infos[_id].point.y

                if self.last_time == 0:
                    acc = 0
                    
                else:
                    origin_point = [self.last_pose_x, self.last_pose_y]
                    dist = np.sqrt((pos_x - origin_point[0])**2 + (pos_y - origin_point[1])**2)
                    delta_t = rospy.Time.now().to_sec() - self.last_time
                    acc = dist / delta_t

                self.last_pose_x = pos_x
                self.last_pose_y = pos_y
                self.last_time = rospy.Time.now().to_sec()

                object_infos_temp = self.object_infos[1, 1:]
                object_infos_temp = np.append(object_infos_temp, np.array(acc).reshape(1), axis=0)
                self.object_infos[1, :] = object_infos_temp
                

                mean = np.mean(self.object_infos[1, :])
                maximum = np.max(self.object_infos[1, :])

                # print(mean, maximum)

                pedestrain_marker = Marker()
                pedestrain_marker.header.frame_id = 'map'
                pedestrain_marker.header.stamp = rospy.Time()
                pedestrain_marker.id = 7
                pedestrain_marker.type = pedestrain_marker.CUBE
                pedestrain_marker.action = pedestrain_marker.ADD
                pedestrain_marker.scale.x = 0.4
                pedestrain_marker.scale.y = 0.4
                pedestrain_marker.scale.z = 0.4
                pedestrain_marker.color.a = 1
                
                moving_threshold = 0.2
                if mean <= moving_threshold:
                    object_state = 'Not Moving'
                    pedestrain_marker.color.r = 0
                    pedestrain_marker.color.g = 1
                    pedestrain_marker.color.b = 0

                else:
                    object_state = 'Moving'
                    pedestrain_marker.color.r = 1
                    pedestrain_marker.color.g = 0
                    pedestrain_marker.color.b = 0

                pedestrain_marker.pose.position.x = pos_x
                pedestrain_marker.pose.position.y = pos_y
                pedestrain_marker.pose.position.z = 0
                pedestrain_marker.lifetime = rospy.Duration.from_sec(0.5)
                pedestrian_markers.markers.append(pedestrain_marker)

                # print(object_state)

            try:
                self.marker_pub.publish(pedestrian_markers)

            except:
                pass

        else:
            print("None")
            self.object_infos = np.zeros((10, 10), dtype=float)
            self.last_pose_x = 0
            self.last_pose_y = 0
            self.last_time = 0

            pedestrian_markers.markers = []
            

if __name__ == '__main__':
    rospy.init_node('object_info_manager', anonymous=False)
    object_info_manager = ObjectInfoManager()
    rospy.spin()
