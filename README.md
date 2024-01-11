
![Alt text](fig/LinK3D_video.gif)
LinK3D is develped for the real-time 3D feature extraction and matching of LiDAR point cloud, which generates accurate point-to-point matching results in real time. The LinK3D paper **"LinK3D: Linear Keypoints Representation for 3D LiDAR Point Cloud"** (**[PDF](https://arxiv.org/pdf/2206.05927.pdf)**) has been accepted by IEEE Robotics and Automation Letters (**RA-L**). The core idea of LinK3D is derived from a very simple principle: representing the current keypoint with its neighboring keypoints. The LinK3D descriptor is represented by a 180-dimensional vector. It can be potentially extended to 3D registration and LiDAR odometry. We also proposed the real-time place recognition algorithm BoW3D based on LinK3D. For more details about BoW3D, please refer to our paper **"BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM"** in IEEE Robotics and Automation Letters (**RA-L**) (**[PDF](https://arxiv.org/pdf/2208.07473.pdf)**).

<div align=center><img width="1800" height="340" src="fig/coreIdea.png"/></div>


## 1. Publication
If you use the code in an academic work, please cite:

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
Before compile the package, in the main function of Example.cpp file, you should replace the dataset path with the file path in your computer. 

    cd ~/catkin_ws/src
    git clone https://github.com/YungeCui/LinK3D/
    cd ..
    catkin_make -j8
    source devel/setup.bash

## 4. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER.
