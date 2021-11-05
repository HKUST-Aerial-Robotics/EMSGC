# EMSGC: Event-based Motion Segmentation with Spatio-Temporal Graph Cuts

**EMSGC** provides a solution to identify independently moving objects acquired with an event-based camera, i.e., to solve the event-based motion segmentation problem. The problem is cast as an energy minimization one involving the fitting of multiple motion models. EMSGC jointly solves two subproblems, namely eventcluster assignment (labeling) and motion model fitting, in an iterative manner by exploiting the structure of the input event data in the form of a spatio-temporal graph.

Please refer to the **[EMSGC Project Page](https://sites.google.com/view/emsgc)** for more detailed information and for testing event data.

**Videos**

[![VIDEO EMSGC](https://img.youtube.com/vi/ztUyNlKUwcM/2.jpg)](https://youtu.be/ztUyNlKUwcM)

## Publication

This is the code for the IEEE TNNLS paper:

* **[Event-based Motion Segmentation with Spatio-Temporal Graph Cuts](https://arxiv.org/pdf/2012.08730.pdf)**, 
*Yi Zhou, Guillermo Gallego, Xiuyuan Lu, Siqi Liu and Shaojie Shen*, 
IEEE Transactions on Neural Network and Learning Systems (TNNLS) 2021.

* **[Event-based Motion Segmentation by Cascaded Two-Level Multi-Model Fitting]()**, 
*Event-based Motion Segmentation by Cascaded Two-Level Multi-Model Fitting*, IROS 2021.

If you use any of this code, please cite the following publication:

```bibtex
@Article{Zhou21tnnls,
  title   = {Event-based Motion Segmentation with Spatio-Temporal Graph Cuts},
  author  = {Zhou, Yi and Gallego, Guillermo and Lu, Xiuyuan and Liu, Siqi and Shen, Shaojie},
  journal = {IEEE Transactions on Neural Network and Learning Systems},
  year    = {2021}
}
```

Also note that the implementation of event warping and contrast maximization is based on **[dvs_global_flow](https://github.com/tub-rip/dvs_global_flow_skeleton)**. Please cite the corresponding publications if you use them.

# 1. Installation
We have tested our code on machines with the following configurations
* Ubuntu 18.04.5 LTS + ROS melodic + gcc 9.4 (enalbes std::filesystem in C++17) + cmake (>=3.10) + OpenCV 3.2
* ...

## 1.1 Driver Installation
To work with event cameras, in particular the Dynamic Vision Sensors (DVS/DAVIS), you need to install some drivers. 
Please follow the instructions (steps 1-9) at [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) before moving on to the next step. 
Note that you need to replace the name of the ROS distribution with the one installed on your computer.
We use catkin tools to build the code. You should have it installed during the driver installation.

## 1.2 Dependencies Installation

You should have created a catkin workspace in Section 1.1. If not, please go back and create one.

**Clone this repository** into the `src` folder of your catkin workspace. In a terminal, run:

    cd ~/catkin_ws/src 
    git clone https://github.com/HKUST-Aerial-Robotics/EMSGC.git

Dependencies are specified in the file [dependencies.yaml](dependencies.yaml). They can be installed with the following commands from the `src` folder of your catkin workspace:

    cd ~/catkin_ws/src
    sudo apt-get install python3-vcstool
    vcs-import < EMSGC/dependencies.yaml

The previous command should clone the repositories into folders called *catkin_simple*, *glog_catkin*, *gflags_catkin*, *minkindr*, etc. inside the `src` folder of your catkin workspace, at the same level as this repository (EMSGC).

You may need `autoreconf` to compile glog_catkin. To install `autoreconf`, run

    sudo apt-get install autoconf

Note that above command may change on different version of Ubuntu.
Please refer to https://askubuntu.com/a/269423 for details.

**yaml-cpp** is only used for loading calibration parameters from yaml files:

    cd ~/catkin_ws/src 
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp
    mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON ..
    make -j

Other ROS dependencies should have been installed in Section 1.1.
If not by accident, install the missing ones accordingly.
Besides, you also need to have `OpenCV` (3.2 or later) and `Eigen 3` installed.

## 1.3 Installation
After cloning this repository, as stated above (reminder)

    cd ~/catkin_ws/src 
    git clone https://github.com/HKUST-Aerial-Robotics/EMSGC.git

run

    catkin build emsgc
    source ~/catkin_ws/devel/setup.bash

# 2. Usage

First you need to download rosbag files from the [EMSGC Project Page](https://sites.google.com/view/emsgc).

Once you have the data ready, go to the launch file and adapt the paths to your setup, including:
  - path to calibration_file
  - path to config_file
  - path to event_data
  - path to where results are saved

Then run e.g.,

    $ roslaunch emsgc box_seq00.launch

# 3. Parameters

- num_events_per_image: The number of events involved in the Space-Time volume.
- timestamp_begin: Starting timestamp.
- timestamp_end: Ending timestamp.
- timestamp_step: Determines the timestamp at which the motion segmentation is performed.
- DisplayResult: Set "True" for saving results to local files.
- SaveResult: Set "True" for displaying result with OpenCV API.
- advanceNum: Number of estimation skipped.

### MRF Parameters
- LAMBDA_data: Coefficient of data term.
- LAMBDA_smooth: Coefficient of Potts model.
- LAMBDA_label: Coefficient of MDL term.
- NUM_ITER_SEGMENTATION: Number of interation for continuous update.
- NUM_ITER_LABELING: Number of interation for discrete update (alpha-expension graph cut).

### Initialization
- HybridModelCombination: 0->Only2D, 1->Only3D, 2->Only4D, 3->Both2Dand3D, 4->Both2Dand4D, 5->Both3Dand4D, 6->All2D3D4D
- BracketRow, BracketCol: Space-Time Volume division (initialization) parameters.
- Affine_trans2d_max: Maximum translation.
- Affine_scale_max: Maximum scale.
- Affine_theta_max: Maximum theta.
- division_exponent: Binary search parameter.

You may set the verbosity / printing level in the command line directly, by settign the value of variable `GLOG_v` (>= 0). Example:

    env GLOG_v=2 roslaunch emsgc box_seq00.launch

# 5. Data
Data can be downloaded from the [Project page](https://sites.google.com/view/emsgc).

# 6. FAQs
- Q1: The results provided in `/result` do not look exactly the same as those shown in the paper.

  A1: This is due to that the segmentation is not performed at exactly the same timestamp as in the paper. Why? Each dataset has its own way of restoring/parsing the timestamp for evaluation. To make the release as compact as possible, we provide only a unified interface as an example. Besides, the color system (HSV) is enabled by default in the implementation. You may switch to RGBC_rendering to get the same color system as in the paper.