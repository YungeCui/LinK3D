
![Alt text](fig/LinK3D_video.gif)
LinK3D is develped for the real-time 3D feature extraction and matching of LiDAR point cloud, which generates accurate point-to-point matching results in real time. The LinK3D paper **"LinK3D: Linear Keypoints Representation for 3D LiDAR Point Cloud"** (**[PDF](https://arxiv.org/pdf/2206.05927.pdf)**) has been accepted by IEEE Robotics and Automation Letters (**RA-L**). The core idea of LinK3D is derived from a very simple principle: representing the current keypoint with its neighboring keypoints. The LinK3D descriptor is represented by a 180-dimensional vector. It can be potentially extended to 3D registration and LiDAR odometry. We also proposed the real-time place recognition algorithm BoW3D based on LinK3D. For more details about BoW3D, please refer to our paper **"BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM"** in IEEE Robotics and Automation Letters (**RA-L**) (**[PDF](https://arxiv.org/pdf/2208.07473.pdf)**).

<div align=center><img width="1800" height="340" src="fig/coreIdea.png"/></div>


## 1. Publication
If you use the code in an academic work, please cite:

    @article{cui2022link3d,
      title={Link3d: Linear keypoints representation for 3d lidar point cloud},
      author={Cui, Yunge and Zhang, Yinlong and Dong, Jiahua and Sun, Haibo and Zhu, Feng},
      journal={arXiv preprint arXiv:2206.05927},
      year={2022}
    }

    @ARTICLE{9944848,
      author={Cui, Yunge and Chen, Xieyuanli and Zhang, Yinlong and Dong, Jiahua and Wu, Qingxiao and Zhu, Feng},
      journal={IEEE Robotics and Automation Letters}, 
      title={BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM}, 
      year={2023},
      volume={8},
      number={5},
      pages={2828-2835},
      doi={10.1109/LRA.2022.3221336}}

## 2. Prerequisites
We have tested the library in Ubuntu 16.04 and 20.04. A computer with an Intel Core i7 will ensure the real-time performance and provide stable and accurate results.
 
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) for Ubuntu 16.04 and [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu 20.04.
- [PCL(>=1.7)](https://github.com/PointCloudLibrary/pcl)
- [OpenCV](https://github.com/opencv/opencv)
- [Eigen 3](https://eigen.tuxfamily.org/dox/)

## 3. Build LinK3D
Clone the repository and catkin_make: 

    cd ~/catkin_ws/src
    git clone https://github.com/YungeCui/LinK3D/
    cd ..
    catkin_make -j8
    source devel/setup.bash

If the compilation is not successful, perhaps you should adjust the CMAKE_CXX_STANDARD in the CMakeList.txt to match the C++ version on your computer.

## 4. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. The program supports reading the point clouds in the **.bin** format of the KITTI dataset. Change the **"dataset_path"** and **"scan_line"(default:64)** params in the **run_bin.launch** file, and save the changes. Finally, execute the command:

    roslaunch LinK3D run_bin.launch

## 5. Rosbag Example 
1).  The program also sopports receiving the **Rosbag** messages. The [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) dataset is collected by a Velodyne VLP-32C LiDAR. The [Stevens-VLP16-Dataset](https://github.com/TixiaoShan/Stevens-VLP16-Dataset) is collected by a Velodyne VLP-16 LiDAR. The two datasets provide the point cloud in the form of **".bag"** format. To run the M2DGR dataset, you should change the **"scan_line"** param to **32** in the **run_rosbag.launch** file, and save the change. To run the Stevens dataset, you should change the **"scan_line"** param to **16** in the **run_rosbag.launch** file, and save the change. Then, execute the command:

    roslaunch LinK3D run_rosbag.launch

2).  Play the bag file corresponding to the **"scan_line"** param:

    rosbag play *.bag
