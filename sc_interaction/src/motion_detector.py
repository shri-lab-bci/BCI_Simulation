#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sc_interaction.msg import ObjectInfo, ObjectInfos


class MotionDetector(object):
    """
    """

    def __init__(self):
        # 글로벌 변수 설정
        self.object_infos = np.zeros((10, 10), dtype=float)
        self.last_pose_x = 0
        self.last_pose_y = 0
        self.last_time = 0

        # 구독 설정
        rospy.Subscriber("/recognition/object_infos_tf", ObjectInfos, self.update_info)
        
        # 발행 설정
        self.marker_pub = rospy.Publisher("recognition/object_markers", MarkerArray, queue_size=10)
        self.obejct_infos_state_pub = rospy.Publisher("recognition/object_infos_st", ObjectInfos, queue_size=10)

    def update_info(self, data):
        """
        """
        object_infos = data
        object_markers = MarkerArray()

        object_infos_state = ObjectInfos()
        object_infos_state.header = object_infos.header
        
        if len(object_infos.object_infos) != 0:
            for _id in range(len(object_infos.object_infos)):
                object_id = int(object_infos.object_infos[_id].id)

                pos_x = object_infos.object_infos[_id].point.x
                pos_y = object_infos.object_infos[_id].point.y

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

                object_marker = Marker()
                object_marker.header.frame_id = 'map'
                object_marker.header.stamp = rospy.Time()
                object_marker.id = 7
                object_marker.type = object_marker.CUBE
                object_marker.action = object_marker.ADD
                object_marker.scale.x = 0.4
                object_marker.scale.y = 0.4
                object_marker.scale.z = 0.4
                object_marker.color.a = 1
                
                moving_threshold = 0.2
                if mean <= moving_threshold:
                    object_state = 'Not Moving'
                    object_marker.color.r = 0
                    object_marker.color.g = 1
                    object_marker.color.b = 0

                else:
                    object_state = 'Moving'
                    object_marker.color.r = 1
                    object_marker.color.g = 0
                    object_marker.color.b = 0

                object_marker.pose.position.x = pos_x
                object_marker.pose.position.y = pos_y
                object_marker.pose.position.z = 0
                object_marker.lifetime = rospy.Duration.from_sec(0.5)
                object_markers.markers.append(object_marker)

                # print(object_state)

                object_info_state = ObjectInfo()
                object_info_state.id = object_infos.object_infos[_id].id
                object_info_state.point = object_infos.object_infos[_id].point
                object_info_state.state.data = object_state

                object_infos_state.object_infos.append(object_info_state)


            try:
                self.marker_pub.publish(object_markers)
                self.obejct_infos_state_pub.publish(object_infos_state)
                # print(object_infos_state)
                # print(type(object_markers))

            except:
                print("here")
                pass

        else:
            print("None")
            self.object_infos = np.zeros((10, 10), dtype=float)
            self.last_pose_x = 0
            self.last_pose_y = 0
            self.last_time = 0

            object_markers.markers = []
            object_infos_state.object_infos = []
            

if __name__ == '__main__':
    rospy.init_node('motion_detector', anonymous=False)
    motion_detector  = MotionDetector()
    rospy.spin()
