# Program Guideline

**Date:** 2025/12/27

**Author:** Joungbin Choi, Hyeongyu Seo, Inyeop Kim



**본 문서는 Robot Automation Project 보고서의 명령어 매뉴얼 부분을 한국어로 작성한 것이다.**



## 파일 구조

```
catkin_ws
  |- build
  |- devel
  |- src
  |----|- CMakeLists.txt
  |----|- ur5e_python
  |----|----|- msg
  |----|----|----|- object_info.msg
  |----|----|----|- ...
  |----|----|- src
  |----|----|----|- dual_network.py
  |----|- indy_driver
  |----|----|- msg
  |----|----|----|- comp_wind_info.msg
  |----|----|----|- decomp_wind_info.msg
  |----|----|----|- bracket_info.msg
  |----|----|----|- comp_pose_info.msg
  |----|----|----|- decomp_pose_info.msg
  |----|----|----|- cell_state_info.msg
  |----|----|----|- process_info.msg
  |----|----|- src
  |----|----|----|- camera.py
  |----|----|----|- Mid_image_display.py
  |----|----|----|- Mid_image_display2.py
  |----|----|----|- Mid_image_display3.py
  |----|----|----|- Mid_image_display4.py
  |----|----|----|- Mid_image_display5.py
  |----|----|----|- Mid_image_display6.py
  |----|----|----|- Mid_communication.py
  |----|----|----|- Mid_project.py
  |----|----|----|- CmakeLists.txt
  |----|----|----|- package.yml
  |----|- MachineVision
  |----|----|- AcquireAndDisplay_Final.py
  |----|----|- best.pt
  |----|----|- train
  |----|----|----|- ...
```



## 소프트웨어 설정

- OS: Linux Ubuntu 20.04

- ROS: ROS Noetic

- Virtual Environment: Anaconda

  - Machine Vision 용 가상환경 생성 및 설정은 [링크](https://github.com/Joungbin-C/IAIA_Robot_Automation_25-2/blob/main/src/MachineVision/README.md)를 따른다
    - ```bash
      conda create -n pyspin python=3.10 -y
      conda activate pyspin
      python -m ensurepip
      python -m pip install --upgrade pip
      
      # 설치한 버전으로 변경
      cd C:\path\to\spinnaker_python_wheel
      pip install spinnaker_python-3.x.x.x-cp38-cp38-linux_x86_64.whl 
      
      pip install numpy==2.2.6
      pip install opencv-python-headless==4.10.0.84
      pip install ultralytics==8.3.232
      pip install PyQt5==5.15.11
      pip install Pillow==9.2.0
      ```
- Additional Ubuntu utility programs

  - terminator
  - Visual Code

추가적인 소프트웨어 설정은 [링크](https://github.com/hyKangHGU/Industrial-AI-Automation_HGU/tree/main/tutorial/ubuntu)를 따른다.



## Terminal 명령어 모음

#### UR5e 실행

우선 UR5e 로봇을 실행 전 my_robot_calibration.yaml을 홈 파일에 저장한 후 다음과 같은 명령어를 터미널에 실행한다.
```
chmod+x/~my_robot_calibration.yaml
```

그리고 아래 노드를 순서대로 실행한다.
```
cd~/catkin_ws
catkin_make
roscore

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2

roslaunch ur5e_rg2_moveit_config move_group.launch

chmod +x ~/catkin_ws/src/ur_python/src/camera.py
rosrun 22000167_Kiminyeop_ur_python camera.py

chmod +x ~/catkin_ws/src/ur_python/src/yellow_bracket_detection_node.py
rosrun 22000167_Kiminyeop_ur_python yellow_bracket_detection_node.py

chmod +x ~/catkin_ws/src/ur_python/src/filled_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python filled_cellbox_detection_node.py

chmod +x ~/catkin_ws/src/ur_python/src/empty_composite_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python empty_composite_cellbox_detection_node.py

chmod +x ~/catkin_ws/src/ur_python/src/filled_composite_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python filled_composite_cellbox_detection_node.py

chmod +x ~/catkin_ws/src/ur_python/src/empty_cell_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python empty_cell_cellbox_detection_node.py

chmod +x ~/catkin_ws/src/ur_python/src/winding_detect_node.py
rosrun 22000167_Kiminyeop_ur_python winding_detect_node.py

chmod +x ~/catkin_ws/src/ur_python/src/composite_detection.py
rosrun 22000167_Kiminyeop_ur_python composite_detection.py

chmod +x ~/catkin_ws/src/ur_python/src/cell_detection.py
rosrun 22000167_Kiminyeop_ur_python cell_detection.py
```

**아두이노 실행은 아래와 같은 절차를 따른다.**
Arduino 1.8.19를 설치한 후, 상단 도구 모음에서 보드가 **Arduino Uno**로 설정되어 있고 포트가 **/dev/tty/ACM0**인지 확인한다.  
설정을 확인한 뒤 업로드를 진행하고, 업로드가 완료되면 Arduino 창을 닫는다.

**마지막으로 VS code에  들어가서 main.py를 실행한다.**

#### Indy10 실행

아두이노와 카메라를 연결할 때는 터미널에서 다음 노드들을 순서대로 실행하고, 코드(camera.py, Mid_project.py)에서 각 장치의 인덱스를 확인한다.
```bash
roscore

rosrun indy_driver camera.py

rosrun indy_driver Mid_image_display.py

rosrun indy_driver Mid_image_display2.py

rosrun indy_driver Mid_image_display3.py

rosrun indy_driver Mid_image_display4.py

rosrun indy_driver Mid_image_display5.py

rosrun indy_driver Mid_image_display6.py

rosrun indy_driver Mid_communication.py

roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.8
rosrun indy_driver Mid_project.py
```


#### Machine Vision 실행

```bash
conda activate pyspin
cd ~/catkin_ws/MachineVision/AcquireAndDisplay_Final.py
python AcquireAndDisplay_Final.py
```

