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
2)RealSense D435 Plugin
:<https://www.clearpathrobotics.com/assets/guides/noetic/jackal/additional_sim_worlds.html>
```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description ros-$ROS_DISTRO-gazebo-plugins
```
Then create your customized URDF file, for example $HOME/Desktop/realsense.urdf.xacro .
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
3)Change the map file


## 3. Usage
Use upper 3.x python version
```bash
export JACKAL_URDF_EXTRAS=$HOME/Desktop/realsense.urdf.xacro
```
```bash
roslaunch jackal_gazebo jackal_world.launch
```
or
```bash
python3 src/main.py
```



## 4. Markdown Syntax


### 4-1. 헤더(Header)


# 헤더 크기 (h1) 
## 헤더 크기 (h2) 
### 헤더 크기 (h3) 
#### 헤더 크기 (h4) 
##### 헤더 크기 (h5) 
###### 해더 크기 (h6)


### 4-2. 목록(List)


Unordered 
* Item 1 
* Item 2 
    * Item 2a 
    * Item 2b 

Ordered 
1. Item 1 
1. Item 2 
1. Item 3 
    1. Item 3a 
    1. Item 3b

### 4-3. 이미지(Images)


첫번째 방법  
![Github logo](/assets/images/markdown_logo.jpg)  
Format: ![이미지 alt명](url 링크)  


두번째 방법  
<a href="#"><img src="https://github.com/hyeonukbhin/template_repository/blob/master/assets/images/markdown_syntax.jpg" width="400px" alt="sample image"></a>  
Format: img 태그 사용 - 이미지경로는 상대경로 or 절대경로  

### 4-4. 하이퍼링크(HyperLink)


[GitHub](http://github.com "깃허브")

### 4-5. 코드 블록(Code Blocks)


```javascript 
function test() { 
 console.log("hello world!"); 
} 
```

코드 스타일은 링크 참조 [GitHub](http://haroopress.com/post/fenced-code-block/, "코드 블럭 스타일")


### 4-6. 인용 상자(Blockquotes)


As Grace Hopper said: 

> I’ve always been more interested. 
> in the future than in the past.


### 4-7. 강조(Emphasis)


*This text will be italic* 
_This will also be italic_ 

**This text will be bold** 
__This will also be bold__ 

*You **can** combine them*


### 4-8. 테이블(Tables)


First Header | Second Header 
------------ | ------------- 
Content cell 1 | Content cell 2 
Content column 1 | Content column 2

### 4-9. 체크박스(Check Boxs)


- [x] this is a complete item 
- [ ] this is an incomplete item 
- [x] @mentions, #refs, [links](), **formatting**, and <del>tags</del> supported 
- [x] list syntax required (any unordered or ordered list supported)


### 4-10. 인라인 코드(Inline Code)


문단 중간에 `Code`를 넣을 수 있습니다. 
예를 들어 `printf("hello world!");` 이런 식으로 들어갑니다.

### 4-11. 수평선(Horizontal Line)


--- 
*** 
___

### 4-12. 탈출 문자(Backslash Escapes)


＼*literal asterisks＼* 
*literal asterisks* 
__＼*＼*Text＼*＼*__ 
_＼_Tom＼__


### 4-13. Github 이모지(EMOJI)


GitHub supports emoji! 
:+1: :sparkles: :camel: :tada: 
:rocket: :metal: :octocat:

### 4-14. 뱃지(Badge)


작성 예시 
<https://img.shields.io/badge/license-mit-green.svg"> 
https://img.shields.io/badge/--.svg 

APM: /apm/l/:packageName.svg 
AUR license: /aur/license/:packageName.svg
