# EMSGC
This repository maintains the implementation of the paper "Event-based Motion Segmentation withSpatio-Temporal Graph Cuts".

**Videos**

[![IMAGE ALT TEXT HERE](https://youtu.be/Ev7lQOhqiUk/1.jpg)](https://youtu.be/Ev7lQOhqiUk)

### Related Publications

* **[Event-based Motion Segmentation with Spatio-Temporal Graph Cuts](https://arxiv.org/pdf/2012.08730.pdf)**, 
*Yi Zhou, Guillermo Gallego, Xiuyuan Lu, Siqi Liu and Shaojie Shen*, submitted to IEEE 
Transactions on Neural Network and Learning Systems (T-NNLS) 2021.

* **[Event-based Motion Segmentation by Cascaded Two-Level Multi-Model Fitting]()**, 
*Event-based  Motion  Segmentation  by  CascadedTwo-Level  Multi-Model  Fitting*, IROS 2021.


# 1. Installation
We have tested ESVO on machines with the following configurations
* Ubuntu 18.04.5 LTS + ROS melodic + gcc 5.5.0 + cmake (>=3.10) + OpenCV 3.2
* ...
## 1.1 Driver Installation
To work with event cameras, especially for the Dynamic Vision Sensors (DVS/DAVIS), you need to install some drivers. 
Please follow the instructions (steps 1-9) at [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) before moving on to the next step. Note that you need to replace the name of the ROS distribution with the one installed on your computer.
We use catkin tools to build the code. You should have it installed during the driver installation.

## 1.2 Dependencies Installation

You should have created a catkin workspace in Section 1.1. If not, please go back and create one.

**Clone this repository** into the `src` folder of your catkin workspace.

	$ cd ~/catkin_ws/src 
	$ git clone https://github.com/HKUST-Aerial-Robotics/EMSGC.git

Dependencies are specified in the file [dependencies.yaml](dependencies.yaml). They can be installed with the following commands from the `src` folder of your catkin workspace:

	$ cd ~/catkin_ws/src
	$ sudo apt-get install python3-vcstool
	$ vcs-import < EMSGC/dependencies.yaml

The previous command should clone the repositories into folders called *catkin_simple*, *glog_catkin*, *gflags_catkin*, *minkindr*, etc. inside the `src` folder of your catking workspace, at the same level as this repository (EMSGC).

You may need `autoreconf` to compile glog_catkin. To install `autoreconf`, run

	$ sudo apt-get install autoconf

Note that above command may change on different version of Ubuntu.
Please refer to https://askubuntu.com/a/269423 for details.

**yaml-cpp** is only used for loading calibration parameters from yaml files:

	$ cd ~/catkin_ws/src 
	$ git clone https://github.com/jbeder/yaml-cpp.git
	$ cd yaml-cpp
	$ mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON ..
	$ make -j

Other ROS dependencies should have been installed in Section 1.1.
If not by accident, install the missing ones accordingly.
Besides, you also need to have `OpenCV` (3.2 or later) and `Eigen 3` installed.

## 1.3 ESVO Installation
After cloning this repository, as stated above (reminder)

	$ cd ~/catkin_ws/src 
	$ git clone https://github.com/HKUST-Aerial-Robotics/EMSGC.git

run

	$ catkin build emsgc
	$ source ~/catkin_ws/devel/setup.bash

# 2. Usage

First you need to download rosbag files from the [EMSGC Project Page](https://sites.google.com/view/emsgc-project-page/home).

Once you have the data ready, go to the launch file and re-edit the paths including
- path to calibration_file
- path to config_file
- path to event_data
- path to where results are saved
, then run e.g.,

    
    $ roslaunch emsgc box_seq00.launch

# 3. Parameters

- width, height: Image size
- num_events_per_image: The number of events involved in the ST volume
- contrast_measure: Dispersion metrics for IWE's constrast. 0 -> VARIANCE_CONTRAST, 1 -> MEAN_SQUARE_CONTRAST
- use_polarity: Set "False" to ignore polarity (Only False is supported in this release)
- gaussian_smoothing_sigma: Sigma of the Gaussian for replacing the Dirac function (see Eq.(3) and (4) in the paper)
- verbosity: Set "1" for detailed output
- timestamp_begin: Starting timestamp
- timestamp_end: Ending timestamp
- timestamp_step: Temporal step (determines the timestamp at which the motion segmentation is performed.)
- DisplayResult: Set "True" for saving results to local files.
- SaveResult: Set "True" for displaying result with OpenCV API

### MRF Parameters
- LAMBDA_data: Coefficient of data term
- LAMBDA_smooth: Coefficient of Potts model
- LAMBDA_label: Coefficient of MDL term
- NUM_ITER_SEGMENTATION: Number of interation for continuous update
- NUM_ITER_LABELING: Number of interation for discrete update (alpha-expension graph cut)

### Initialization
- HybridModelCombination: 0->Only2D, 1->Only3D, 2->Only4D, 3->Both2Dand3D, 4->Both2Dand4D, 5->Both3Dand4D, 6->All2D3D4D
- BracketRow, BracketCol: Number of divisions in each dimension.
- Affine_trans2d_max: Maximum translation
- Affine_scale_max: Maximum scale
- Affine_theta_max: Maximum theta
- division_exponent: Times of binary search.

# 5. Datasets (TO DELETE)
Data can be downloaded from the [here](https://drive.google.com/drive/folders/1KB4oUOQcPF9v1u9GEaCMowPkRHyVxodc?usp=sharing).

