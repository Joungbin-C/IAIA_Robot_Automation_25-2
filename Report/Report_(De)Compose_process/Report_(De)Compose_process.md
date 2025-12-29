# Project 2: Robot Automation

Industrial AI & Automation 2025

**Name: ** Seo HyeonGyu

**Date:** 2025/12/25

**Github**: [Link](https://github.com/Joungbin-C/IAIA_Robot_Automation_25-2/tree/main)

[TOC]

____

## I. Introduction

With the rapid growth of the electric vehicle (EV) market, the secondary battery industry has emerged as a key strategic sector. It is projected that by 2030, electric vehicles will account for more than 40% of the global automotive market, leading to a significant increase in battery production demand. However, conventional battery cell assembly and disassembly processes remain highly dependent on manual labor, resulting in limitations in productivity, process consistency, and quality control. In particular, as the importance of battery recycling and safe disassembly increases, there is a growing demand for intelligent automation systems capable of handling precise and complex operations beyond simple assembly tasks.

This project aims to address these challenges by developing an **automated battery cell winding process using industrial robots integrated with machine vision and artificial intelligence technologies**. By combining robotic manipulation, sensor-based detection nodes, state diagram–based control logic, and camera-based recognition of battery cells and winding components, the project seeks to transform a labor-intensive process into a high-speed, high-precision automated system. Through this approach, the feasibility of improving productivity, reliability, and safety in battery manufacturing and recycling processes is investigated.

<img src="img\background.png" width="60%">

___________

## II. Problem Statement

### 1. Problems

1. Minor process deviations create potential defects, leading to battery failures and safety hazards.

2. In mass production, significant process variation creates challenges in ensuring uniformity.

3. Conventional defect inspection methods are slow and only detect surface defects.

<img src="img\problem.png" width="60%">

### 2. Projective Objectives

The main objectives of this project are as follows:

1. **Implementation of Automated Assembly and Disassembly of Battery Cell Winding Processes**
    To automate the assembly and disassembly of battery cells and winding components using industrial robots, thereby minimizing human error and improving repeatability.
2. **Development of a Machine Vision–Based Recognition System**
    To detect battery cell positions, orientations, and conditions (normal or defective) using camera-based vision systems and AI models, and to integrate the recognition results into the robot control system.
3. **Design of Robot Control Logic and State Diagrams**
    To design structured state diagrams that manage process flow and ensure stable coordination between sensor detection nodes and robotic actions.
4. **Verification of Process Reliability and Performance**
    To experimentally evaluate the success rate, accuracy, and operational stability of the automated system and assess its applicability to real industrial environments.

### 3. Expected Outcomes

The expected outcomes of this project are as follows:

1. **Improved Productivity and Operational Efficiency**
    By replacing manual labor with robotic automation, continuous operation beyond traditional working hours becomes possible, significantly increasing overall productivity.
2. **Enhanced Accuracy and Quality Consistency**
    The automated system is expected to achieve a high success rate in assembly and disassembly tasks, while the machine vision system enables reliable identification of normal and abnormal battery cells, excluding specific inclusion cases.
3. **Reduced Labor Dependency and Improved Workplace Safety**
    Automating repetitive and potentially hazardous battery handling processes reduces worker exposure to risk and lowers dependence on skilled manual labor.
4. **Scalability for Battery Recycling and Future Smart Factory Applications**
    The proposed system can be extended to battery disassembly and recycling processes, providing a technological foundation for advanced smart factory and sustainable battery production systems.

### 4. Evaluation Index

For each process, 100 iterations of the test are applied to determine the process success rate threshold for each node and set this as the target.

| Process                    | Target                                                       |
| -------------------------- | ------------------------------------------------------------ |
| **UR5 Robot Operation**    | - Bracket detection success rate ≥ 98% <br />- Cell box detection success rate = 100% <br />- Pick and Place success rate ≥ 95% <br />- Stopper linkage success rate = 100% <br />- Assembly/Disassembly process completion time ≤ 1 minute |
| **Indy10 Robot Operation** | - Cell orientation detection success rate = 100% <br />- Foreign object detection success rate ≥ 95% <br />- Defective cell detection success rate ≥ 95% <br />- Stopper linkage success rate = 100% <br />- Winding assembly/disassembly process completion time ≤ 1 minute |
| **Defect Detection**       | - Defect detection success rate (cylindrical surface) ≥ 95% <br />- Defect detection success rate (flat surface) ≥ 80% |

_______

## III. Requirements

### 1. Hardware list

**Co-Robot**

- UR-5e
- Indy-10

**Grippers**

- 2-finger gripper
- Vacuum gripper
- Screw-driver

**Material**

* Wind
* Cell
* Bracket

<img src="img\material.png" width="60%">

### 2. Software list

* Ubuntu 20.04
* Python 3.10.8
* Opencv
* Numpy

______________

## IV. Procedure

This report covers the camera-based cell and winding **disassembly and assembly automation process (UR robot)**.
- [disassembly and assembly automation](https://github.com/Joungbin-C/IAIA_Robot_Automation_25-2/tree/main/Report/Report_(Dis)assembly_Automation_Process/Report_(Dis)Assembly_Automation.md)

Process recognition-based **pick and place automation (Indy10)** and deep learning-based **cell defect detection** are covered in the following links.

* [pick and place automation]()
* [cell defect detection](https://github.com/shg0873/IAIA/tree/main/Project/Project2_RobotAutomation/Report)

### 1. Overview

The assembly process consists of two main stages: the winding process, and the defective cell detection process. During the winding process, the system identifies the bracket and the empty battery cell. Once the relative coordinates of the windings are detected, the components are assembled. This process also includes a defective cell detection mechanism, which classifies defective products based on flags received during operation.

The disassembly process focuses on removing the windings from the finished battery cell. Like the assembly process, this step also requires the detection of the bracket and the finished battery cell. Once the system detects the relative coordinates of the windings, the components are disassembled and placed into the empty winding box.

The entire process is illustrated in the flow chart below, and the goal of the entire process is to ensure high reliability through rigorous testing. The goal is to perform 100 repeat tests for each process. Based on these tests, a success rate threshold is established for each node, which serves as the target performance standard for the automated system.

<img src="img\flowchart.png" width="60%">

You need to run the following nodes in the terminal in order and check the **index** of each device in the code when connecting the Arduino and camera.

```python
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



### 2. Detection Node 1: Bracket detecting 

<img src="img\node1.png" width="20%">

The 1st node code is **Mid_image_display2**. This node determines whether a bracket has arrived. The presence of a bracket is determined by calculating the average intensity value using standard image processing techniques, taking advantage of the fact that the conveyor belt is white and the bracket is black. If the average intensity value falls below the threshold (150.0), a flag is set.

<img src="img\node1_display.png" width="18%">

### 3. Detection Node 2: Wind Detecting whether wind combined with cell

<img src="img\node2.png" width="25%">

The 2nd node code is **Mid_image_display4**. At this node, the process is determined by determining whether a winder is attached to the cell. If wind is detected through the classical image processing process below, the process proceeds to the disassembly process. If wind is not detected, the process proceeds to the attachment process.

<img src="img\node2_display.png" width="60%">

### 4. Detection Node 3: Wind number detection during the composing process

<img src="img\node3.png" width="40%">

The 3rd node code is **Mid_image_display1**. This node determines the wind number to be combined during the combination process. Through color detection, if the wind color exceeds a threshold, the box is determined to be filled with wind, and information on which wind number it is is passed to the indy10 robot. Starting from the left, the wind number information is 1, 2, 3 at the top, and 4, 5, 6 at the bottom.

<img src="img\node3_display.png" width="30%">

###  5. Detection Node 4: Calculate the wind center During the composing process

<img src="img\node4.png" width="40%">

The 4th node code is **Mid_image_display3**. In order to capture the wind to be combined with the cell in the combination process at the corresponding node, the relative coordinates of the wind center are calculated. The relative coordinates are obtained using the formula below after classical image processing, and the return value is the relative coordinates of the wind center [cm].

<img src="img\node4_display.png" width="45%">

$$
x_{cm}=-(v-{M\over 2})\cdot {M\over H}
$$

$$
y_{cm}=-(u-{N\over2})\cdot{N\over W}
$$

* W denotes word width, H denotes word height
* N denotes image pixel width, M denotes image pixel height

### 6. Detection Node 5: Calculating the wind center during the decomposition process

<img src="img\node5.png" width="20%">

The 5th node code is **Mid_image_display5**. At this node, the relative coordinates of the wind center are calculated to capture the wind to be separated from the cell during the decomposition process. The relative coordinates are obtained using the following formula after classical image processing, and the return value is the relative coordinates of the wind center [cm]. The formula is the same as for Node 4.

### 7. Detection Node 6: Wind number detection during the decomposition process

<img src="img\node6.png" width="34%">

The 5th node code is **Mid_image_display6**.  This node determines the number at which to place the disassembled wind during the disassembly process. If the wind color is below a threshold through color detection, the box is determined to be empty and information on the number of the wind is passed to the indy10 robot. The wind numbers are 1, 2, 3 at the top, and 4, 5, 6 at the bottom, starting from the left.



____________

## V. Results

The test was performed a total of 100 times. Process time was measured from the moment the stopper contacts the bracket until the stopper opens at the end of the process.

| Part                                              | Target                                                       | Result                    | Goal achievement |
| :------------------------------------------------ | ------------------------------------------------------------ | ------------------------- | ---------------- |
| Detecting part                                    | - Cell orientation detection success rate = 100%             | - 100%                    | O                |
| Pick and Place and Defective<br /> Cell Erase     | - Foreign object detection success rate ≥ 95% <br />- Defective cell detection success rate ≥ 95% | - 98%<br />- 99%          | O                |
| Stopper linkage and process <br />completion time | - Stopper linkage success rate = 100% <br />- Winding assembly/disassembly process <br />completion time ≤ 1 minute | - 100%<br />- Under 57[s] | O                |

_____

## VI. Conclusion


This project developed an automated system for the battery cell winding assembly and disassembly process to address the increasing demand for batteries driven by the growth of the electric vehicle market and to address the productivity and safety issues inherent in existing manual processes. By integrating industrial robots and machine vision technology, we implemented an intelligent process that performs bracket and cell detection, defect detection, assembly, and disassembly. The system's reliability was verified through 100 repeated experiments. The results showed a 100% success rate in cell orientation detection and stopper linkage, and a 98% or higher success rate in foreign matter and defective cell detection, exceeding the targets in all evaluation indicators. Notably, by shortening the process completion time to less than 57 seconds, well below the target of one minute, this system proved to be an effective solution that can dramatically improve production efficiency and work safety in the battery manufacturing and recycling industries.


