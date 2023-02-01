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

> update_object_count 함수 변경

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

> intrinsic 파라미터 변경

```bash
	# Camera_Info 메세지의 K행렬에서 해당값들을 찾아볼수 있음.
	# 카메라 특성값 설정 변경
        #self.aligned_depth_intrin.fx = 617.44921875
        #self.aligned_depth_intrin.fy = 617.110168457
        #self.aligned_depth_intrin.height = 480
        #self.aligned_depth_intrin.width = 640
        #self.aligned_depth_intrin.model = rs.distortion.inverse_brown_conrady
        #self.aligned_depth_intrin.ppx = 328.007507324
        #self.aligned_depth_intrin.ppy = 241.498748779
	
        self.aligned_depth_intrin.fx = 337.2084410968044
        self.aligned_depth_intrin.fy = 337.2084410968044
        self.aligned_depth_intrin.height = 480
        self.aligned_depth_intrin.width = 640
        self.aligned_depth_intrin.model = rs.distortion.none
        self.aligned_depth_intrin.ppx = 320.5
        self.aligned_depth_intrin.ppy = 240.5
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

## 4. Detail
GAZEBO setting

1) jackal robot 초기 위치 설정 

`/opt/ros/noetic/share/jackal_gazebo/launch/jackal_world.launch` 파일 내부 arg x, y, z 값 설정.
```bash
<!-- Spawn Jackal -->
  <include file="$(find jackal_gazebo)/launch/spawn_jackal.launch">
    <arg name="x" value="0" />
    <arg name="y" value="0" />
    <arg name="z" value="1.0" />
    <arg name="yaw" value="0" />
    <arg name="config" value="$(arg config)" />
    <arg name="joystick" value="$(arg joystick)" />
  </include>
```

2) Actor 설정

액터의 원하는 경로와 시간, 자세 등을 설정할 수 있습니다.

- waypoint로 경로를 설정합니다.
	
- time은 액터가 있는 시각을 나타냅니다.
	
- pose는 액터의 위치입니다. x,y,z, gx, gy,gz 순서입니다. 사람은 z축 방향만 설정해주면되기 때문에 gz로 바라보는 각도를 설정해줍니다. 0 ~ pi(3.14)로 나타내고 음수도 표현 가능합니다.
	
- skin 과 animation으로 동작 모션 표현.
	
아래는 사각형으로 모양으로 이동하는 코드입니다.

```bash
<actor name="actor">
        <skin>
          <filename>walk.dae</filename>
        </skin>
        <animation name="walking">
          <filename>walk.dae</filename>
          <interpolate_x>true</interpolate_x>
        </animation>
        <script>
          <trajectory id="0" type="walking">
            <waypoint>
              <time>0</time>
              <pose>5 2 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
              <time>7</time>
              <pose>5 -4.5 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
              <time>8</time>
              <pose>5 -4.5 0 0 0 0</pose>
            </waypoint>
            <waypoint>
              <time>21</time>
              <pose>16 -4.5 0 0 0 0</pose>
            </waypoint>
            <waypoint>
              <time>22</time>
              <pose>16 -4.5 0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
              <time>29</time>
              <pose>16 2 0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
              <time>30</time>
              <pose>16 2 0 0 0 3.14</pose>
            </waypoint>
            <waypoint>
              <time>43</time>
              <pose>5 2 0 0 0 3.14</pose>
            </waypoint>
            <waypoint>
              <time>44</time>
              <pose>5 2 0 0 0 -1.57</pose>
            </waypoint>
          </trajectory>
        </script>
      </actor>

```
## 5. Editing the map

> 전체 맵 편집

gazebo 실행
![Image](https://user-images.githubusercontent.com/86464408/215948844-276bac72-484c-45e9-ad4d-cdf945caacdc.png)

Edit > Model Editor : 모델편집기 실행
![Image](https://user-images.githubusercontent.com/86464408/215949075-33b5f585-331e-4109-b21d-d9f69ae46dcf.png)

추가한 경로에 만들어놓은 맵 파일 불러오기
![Image](https://user-images.githubusercontent.com/86464408/215949073-110e1f67-4e84-46f7-b8dd-8be833694af3.png)
맵 파일이 화면에 표시된다.
![Image](https://user-images.githubusercontent.com/86464408/215949074-23248c36-e426-4b4b-92ec-3462ba3ef77a.png)

추가한 경로에 만들어놓은 모듈(여기선 wall)이나 Simple Shapes 에서 제공하는 도형들을 편집해서 맵에 추가한다.
![Image](https://user-images.githubusercontent.com/86464408/215949079-02d40393-64a4-446b-84a2-28705a46f296.png)
이동
![Image](https://user-images.githubusercontent.com/86464408/215949078-a50218c7-5a05-4457-bb30-54f80a4c058d.png)
회전
![Image](https://user-images.githubusercontent.com/86464408/215949080-aa302c54-c8c3-4078-adfb-e420d6ce8049.png)

File > Save/Save as : 저장
![Image](https://user-images.githubusercontent.com/86464408/215949077-e8177f60-23b1-4f15-93f9-079301b4fd6e.png)
![Image](https://user-images.githubusercontent.com/86464408/215949076-78258c58-a814-48ee-9fc6-05d1e1ced2da.png)

Gazebo 실행 > 저장한 맵 파일 실행(해당 예제에서는 Untitled23)
![Image](https://user-images.githubusercontent.com/86464408/215949082-91a52812-9b83-4d8e-a702-69b8e2613e07.png)

해당 파일을 `jackal_race.world` file에 덮어쓰면 업데이트 완료.(액터같은 요소들은 그대로 두기)
![Image](https://user-images.githubusercontent.com/86464408/215949081-57b30b0c-7625-4d64-8c17-c95a0b3cdbba.png)

> AR 추가

가제보 디렉토리에 png 파일 폴더 디렉토리 추가하기 :  1번 링크 <https://github.com/mikaelarguedas/gazebo_models>

ArUco markers SVG 파일 다운로드 : 2번 링크 <https://chev.me/arucogen/> 

1번 링크에서 git clone 하여 $HOME에 다운받는다.

2번 링크에서 받은 마커를 png 파일로 변환하여 gazebo_models/ar_tags/images 폴더 안에 집어넣는다. : 변환하기 링크 <https://svgtopng.com/>

```bash
cd gazebo_models/ar_tags/scripts
```

```bash
./generate_markers_model.py -i $HOME/gazebo_models/ar_tags/images
``` 

!)만약 directory 오류가 뜰 경우 기본 가제보 디렉토리($HOME/.gazebo/models)를 비워주면 된다.

```bash
cd ~/.gazebo
```

```bash
rm -r models
```

```bash
mkdir models
```
Insert 탭에 가제보 디렉토리에 새로운 경로들이 추가된 것을 볼 수 있다. 
![Image](https://user-images.githubusercontent.com/86464408/215956008-34759b44-b6e4-4794-a100-405568dc07c3.png)

추가한 마커(해당 예제에서는 Marker9번)을 불러와서 맵에 추가해주면 된다. 회전, 이동 편집 가능.
![Image](https://user-images.githubusercontent.com/86464408/215956009-84e740d8-4856-495f-99cd-38b4490b1332.png)
