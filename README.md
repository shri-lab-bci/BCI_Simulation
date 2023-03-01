BCI_Simulation ROS Gazebo
===========================================================

* Jetson 인식 모듈
* Ojbect Deprojector
* Object TF BroadCaster
* L8 시뮬레이션 환경
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

3) Change the map file
`/opt/ros/noetic/share/jackal_gazebo/worlds/jackal_race.world`

Follow the path and fix the `jackal_race.world` file. 

Erase all and paste the `1st_floor_L8.world` file(공유된 파일) in it. (내용만 복붙_just contents)

This map describes L8 1st floor and the last part of the code is related with Actors(Human model).

You can customize the Actor and the objects : 

<https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot>

<http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation>

4) Change the 'object_deprojector.py'

## 4. Usage
```
roslaunch jackal_viz view_robot.launch
roslaunch sc_interaction jackal_sim.launch

```
## 4. Etc.
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

> AR 추가 : 참고 링크 <https://www.youtube.com/watch?v=A3fJwTL2O4g>

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
