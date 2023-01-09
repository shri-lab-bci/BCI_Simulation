BCI_Simulation ROS Gazebo 구현 설명서
===========================================================

Jackal Robot Simulation 환경 구축 
Human 추종 알고리즘

## 1. Description

- ROS Noetic 
- python 3.8.10


## 2. Requirements

1) ROS jackal simulation install
   :<https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html>

```bash
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop-*
```
2) RealSense D435 Plugin
:<https://www.clearpathrobotics.com/assets/guides/noetic/jackal/additional_sim_worlds.html>
```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description ros-$ROS_DISTRO-gazebo-plugins
```
Then create your customized URDF file, for example `$HOME/Desktop/realsense.urdf.xacro`.

Put the following in it:
```bash
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <link name="front_realsense" />

  <!--
    The gazebo plugin aligns the depth data with the Z axis, with X=left and Y=up
    ROS expects the depth data along the X axis, with Y=left and Z=up
    This link only exists to give the gazebo plugin the correctly-oriented frame
  -->
  <link name="front_realsense_gazebo" />
  <joint name="front_realsense_gazebo_joint" type="fixed">
    <parent link="front_realsense"/>
    <child link="front_realsense_gazebo"/>
    <origin xyz="0.0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
  </joint>

  <gazebo reference="front_realsense">
    <turnGravityOff>true</turnGravityOff>
    <sensor type="depth" name="front_realsense_depth">
      <update_rate>30</update_rate>
      <camera>
        <!-- 75x65 degree FOV for the depth sensor -->
        <horizontal_fov>1.5184351666666667</horizontal_fov>
        <vertical_fov>1.0122901111111111</vertical_fov>

        <image>
          <width>640</width>
          <height>480</height>
          <format>RGB8</format>
        </image>
        <clip>
          <!-- give the color sensor a maximum range of 50m so that the simulation renders nicely -->
          <near>0.01</near>
          <far>50.0</far>
        </clip>
      </camera>
      <plugin name="kinect_controller" filename="libgazebo_ros_openni_kinect.so">
        <baseline>0.2</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>30</updateRate>
        <cameraName>realsense</cameraName>
        <imageTopicName>color/image_raw</imageTopicName>
        <cameraInfoTopicName>color/camera_info</cameraInfoTopicName>
        <depthImageTopicName>depth/image_rect_raw</depthImageTopicName>
        <depthImageInfoTopicName>depth/camera_info</depthImageInfoTopicName>
        <pointCloudTopicName>depth/color/points</pointCloudTopicName>
        <frameName>front_realsense_gazebo</frameName>
        <pointCloudCutoff>0.105</pointCloudCutoff>
        <pointCloudCutoffMax>8.0</pointCloudCutoffMax>
        <distortionK1>0.00000001</distortionK1>
        <distortionK2>0.00000001</distortionK2>
        <distortionK3>0.00000001</distortionK3>
        <distortionT1>0.00000001</distortionT1>
        <distortionT2>0.00000001</distortionT2>
        <CxPrime>0</CxPrime>
        <Cx>0</Cx>
        <Cy>0</Cy>
        <focalLength>0</focalLength>
        <hackBaseline>0</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>

  <link name="front_realsense_lens">
    <visual>
      <origin xyz="0.02 0 0" rpy="${pi/2} 0 ${pi/2}" />
      <geometry>
        <mesh filename="package://realsense2_description/meshes/d435.dae" />
      </geometry>
      <material name="white" />
    </visual>
  </link>

  <joint type="fixed" name="front_realsense_lens_joint">
    <!-- Offset the camera 5cm forwards and 1cm up -->
    <origin xyz="0.05 0 0.01" rpy="0 0 0" />
    <parent link="front_mount" />
    <child link="front_realsense_lens" />
  </joint>
  <joint type="fixed" name="front_realsense_joint">
    <origin xyz="0.025 0 0" rpy="0 0 0" />
    <parent link="front_realsense_lens" />
    <child link="front_realsense" />
  </joint>
</robot>
```
3) Change the map file
`/opt/ros/noetic/share/jackal_gazebo/worlds/jackal_race.world`

Follow the path and fix the `jackal_race.world` file. 

Erase all and paste the `1st_floor_L8.world` file(공유된 파일) in it. (내용만 복붙_just contents)

This map describes L8 1st floor and the last part of the code is related with Actors(Human model).

You can customize the Actor and the objects : 

<https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot>

<http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation>

4) Change the 'object_deprojector.py'

'object_deprojector.py'파일 내용 수정사항

> subscriber 이름 변경

```bash
#rospy.Subscriber("camera/color/image_raw/compressed", CompressedImage, self.bridge_color_image)
#rospy.Subscriber('camera/depth/image_raw', Image, self.update_depth_image)
rospy.Subscriber("realsense/color/image_raw/compressed", CompressedImage, self.bridge_color_image)
rospy.Subscriber("realsense/depth/image_rect_raw", Image, self.update_depth_image)
```
> update_depth_image 함수 변경

```bash
    def update_depth_image(self, data):
        try:
            #cv_depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1") 인코딩 형식 변경(D435)!!
            cv_depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
```

>update_object_count 함수 변경

```bash
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
                        #median_depth = np.median(depth_array) !)nan값을 포함하기때문에 median->namedian함수 사용
                        median_depth = np.nanmedian(depth_array) 
                        median_depth_index = np.where(depth_array == median_depth)

                        #median_depth = median_depth * 0.001 !)이 라인을 지워야 거리가 올바르게 표시되었습니다.
                        #print(depth_array)
                        #print(median_depth)
                        center_ppx = int((xmin + xmax) / 2)
                        center_ppy = int((ymin + ymax) / 2) 
                        depth_point = rs.rs2_deproject_pixel_to_point(self.aligned_depth_intrin, [center_ppx, center_ppy], median_depth)
			 
                        # Visualization
                        #print(median_depth)
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
            compressed_recognition_image.data = cv2.imencode('.jpg', self.color_image)[1].tobytes()
            self.recognition_image_pub.publish(compressed_recognition_image)

        except CvBridgeError as e:
            pass
```
5) Change the 'Object_tf_br.cpp'

'Object_tf_br.cpp' 파일 내용 수정사항

> tf 로 전사해줄때 parent를 변경

```bash
    #transformStamped.header.frame_id = "camera_link";
    transformStamped.header.frame_id = "front_realsense";
```

## 3. Usage
Use upper 3.x python version

Terminal 1
```bash
export JACKAL_URDF_EXTRAS=$HOME/Desktop/realsense.urdf.xacro

roslaunch jackal_gazebo jackal_world.launch
```

Terminal2 : jetson 인식기 노드 실행 
```bash
ssh name@ip_address

rosrun sc_interaction object_deprojector.py
```

Terminal3 : 제가 제작한 jetson 인식기는 아래 count 정보는 포함시키지 않았기에 따로 pub 해줬습니다.
```bash
rostopic pub -r 4025 /darknet_ros/found_object darknet_ros_msgs/ObjectCount "header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: 'detection'
	count: 1" 
```

Terminal4
```bash
rosrun sc_interaction object_deprojector.py 
```

Terminal5
```bash
rosrun sc_interaction object_tf_br
```

You can see the TF or sensor data by 'roslaunch jackal_viz view_robot.launch'.

