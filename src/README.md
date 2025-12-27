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
  |----|- indy10_python
  |----|----|- msg
  |----|----|- src
  |----|- MachineVision
  |----|----|- AcquireAndDisplay_Final.py
  |----|----|----|----|- ...
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



#### Indy10 실행



#### Machine Vision 실행

```bash
conda activate pyspin
cd ~/catkin_ws/MachineVision/AcquireAndDisplay_Final.py
python AcquireAndDisplay_Final.py
```

