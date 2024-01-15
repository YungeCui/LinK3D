#include <ros/ros.h>

#include <string>

#include <pcl/filters/voxel_grid.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <iostream>
#include "LinK3D_extractor.h"

using namespace std;
using namespace LinK3D_SLAM;


void ransac(vector<pcl::PointXYZ> &vpCurPt, vector<pcl::PointXYZ> &vpLastPt, vector<pair<int, int>> &vMatchedIndex, vector<pair<int, int>> &vTrueMatchedIndex)
{ 
    if(vMatchedIndex.empty()){
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::CorrespondencesPtr corrsPtr (new pcl::Correspondences()); 

    for(int i = 0; i < (int)vMatchedIndex.size(); i++)
    {
        int ptIndex1 = vMatchedIndex[i].first;
        int ptIndex2 = vMatchedIndex[i].second;
        source->push_back(vpCurPt[ptIndex1]);
        target->push_back(vpLastPt[ptIndex2]);

        pcl::Correspondence correspondence(i, i, 0);
        corrsPtr->push_back(correspondence); 
    }

    pcl::Correspondences corrs;
    pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ> Ransac_based_Rejection;
    Ransac_based_Rejection.setInputSource(source);
    Ransac_based_Rejection.setInputTarget(target);
    double sac_threshold = 0.5;
    Ransac_based_Rejection.setInlierThreshold(sac_threshold);
    Ransac_based_Rejection.getRemainingCorrespondences(*corrsPtr, corrs);

    int corrSize = (int)corrs.size();
    for(int i = 0; i < corrSize; i++)
    { 
        pcl::Correspondence corr = corrs[i];
        vTrueMatchedIndex.emplace_back(vMatchedIndex[corr.index_query]); 
    }
}


void ransacForEdgePt(MatPt &matchedEdgePt, vector<pair<PointXYZSCA, PointXYZSCA>> &vTrueMatchedEdgePoint)
{
    if(matchedEdgePt.empty()){
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::CorrespondencesPtr corrsPtr (new pcl::Correspondences()); 

    for(int i = 0; i < (int)matchedEdgePt.size(); i++)
    {
        vector<PointXYZSCA> matchPoint = matchedEdgePt[i];

        pcl::PointXYZI sourcePt;
        sourcePt.x = matchPoint[0].x;
        sourcePt.y = matchPoint[0].y;
        sourcePt.z = matchPoint[0].z;

        pcl::PointXYZI targetPt;
        targetPt.x = matchPoint[1].x;
        targetPt.y = matchPoint[1].y;
        targetPt.z = matchPoint[1].z;

        source->push_back(sourcePt);
        target->push_back(targetPt);

        pcl::Correspondence correspondence(i, i, 0);
        corrsPtr->push_back(correspondence);
    }

    pcl::Correspondences corrs;
    pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZI> Ransac_based_Rejection;
    Ransac_based_Rejection.setInputSource(source);
    Ransac_based_Rejection.setInputTarget(target);
    double sac_threshold = 0.5;
    Ransac_based_Rejection.setInlierThreshold(sac_threshold);
    Ransac_based_Rejection.getRemainingCorrespondences(*corrsPtr, corrs);

    int corrSize = (int)corrs.size();
    set<int> sTrueMatchIndex;
    for(int i = 0; i < corrSize; i++)
    {
        pcl::Correspondence corr = corrs[i];
        int index = corr.index_query;

        sTrueMatchIndex.insert(index);

        PointXYZSCA pt1 = matchedEdgePt[index][0];
        PointXYZSCA pt2 = matchedEdgePt[index][1];

        pair<PointXYZSCA, PointXYZSCA> trueMatchPair = make_pair(pt1, pt2);
        vTrueMatchedEdgePoint.emplace_back(trueMatchPair);
    } 
}

void read_lidar_data(const std::string lidar_data_path, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    if(!lidar_data_file)
    {
        cerr << "Open Error!!! Please Input Right Dataset Path in Runbin.launch File." << endl;
        exit(1);
    }

    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data[0]), num_elements*sizeof(float));
    
    for (std::size_t j = 0; j < lidar_data.size(); j += 4)
    {            
        pcl::PointXYZ point;
        point.x = lidar_data[j];
        point.y = lidar_data[j + 1];
        point.z = lidar_data[j + 2];
        cloud.push_back(point);
    }
}


int nScans = 64;
float minimumRange = 0.1;
float XYDisTh = 0.4;
float XDisTh = 0.3;
float YDisTh = 0.3;
int ptNumTh = 12;
int scanNumTh= 4;
int scoreTh = 3;
std::string dataset_folder;

/*Extract LinK3D features for KITTI dataset*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "LinK3D");
    ros::NodeHandle nh;



    /*The "scan_line" param in the run_bin.launch file should correspond with the used LiDAR.*/
    nh.param<int>("scan_line", nScans, 64);
    cout << "nScans: " << nScans << endl;
    /*The "dataset_path" param in the run_bin.launch file should be replaced with the dataset path in your computer.*/
    nh.param<string>("dataset_path", dataset_folder, "/media/oem/My Passport/dataset/dataset/sequences/00/velodyne/");



    ros::Publisher pubFullCloud1 = nh.advertise<sensor_msgs::PointCloud2>("full_cloud1", 100);
    ros::Publisher pubFullCloud2 = nh.advertise<sensor_msgs::PointCloud2>("full_cloud2", 100);
    ros::Publisher pubKeyPoint = nh.advertise<sensor_msgs::PointCloud2>("key_point", 100);
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("marker", 100);

    LinK3D_SLAM::LinK3D_Extractor *pLinK3D_Extractor = new LinK3D_SLAM::LinK3D_Extractor(nScans, minimumRange, XYDisTh, 
                                                                                         XDisTh, YDisTh, ptNumTh, scanNumTh, scoreTh); 
    
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.pretranslate(Eigen::Vector3d(0, 0, 30));
    

    int count = 0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        count++;
        cout << "count: " << count << endl;

        /*Read last point cloud and extract LinK3D features*/
        std::stringstream last_lidar_data_path;
        last_lidar_data_path << dataset_folder << std::setfill('0') << std::setw(6) << count-1 << ".bin";                            

        pcl::PointCloud<pcl::PointXYZ> last_cloud;
        read_lidar_data(last_lidar_data_path.str(), last_cloud);

        vector<pcl::PointXYZ> lastKeyPoints;
        cv::Mat lastDescriptors;
        vector<int> lastIndex;
        MatPt lastClustered;
        MatPt lastHighSmooth;
        (*pLinK3D_Extractor)(last_cloud, lastKeyPoints, lastDescriptors, lastIndex, lastClustered);

                
        /*Read current point cloud and extract LinK3D features*/
        std::stringstream cur_lidar_data_path;
        cur_lidar_data_path << dataset_folder << std::setfill('0') << std::setw(6) << count << ".bin"; 

        pcl::PointCloud<pcl::PointXYZ> cur_cloud;
        read_lidar_data(cur_lidar_data_path.str(), cur_cloud);

        vector<pcl::PointXYZ> currentKeyPoints;
        cv::Mat currentDescriptors;
        vector<int> currentIndex;
        MatPt currentClustered;
        MatPt curHighSmooth;
        (*pLinK3D_Extractor)(cur_cloud, currentKeyPoints, currentDescriptors, currentIndex, currentClustered);
        

        /*Filter low-smoothness edge keypoints*/
        pLinK3D_Extractor->filterLowSmooth(lastClustered, lastHighSmooth);
        pLinK3D_Extractor->filterLowSmooth(currentClustered, curHighSmooth);
        
        
        /*Match aggregation keypoints*/
        vector<pair<int, int>> vMatched;
        pLinK3D_Extractor->matcher(currentDescriptors, lastDescriptors, vMatched);
        /*Match edge keypoints*/
        MatPt matchedEdgePoint;
        pLinK3D_Extractor->matchEdgePoints(curHighSmooth, lastHighSmooth, currentIndex, lastIndex, matchedEdgePoint, vMatched);
        

        if(vMatched.size() < 5)
        {
            cout << "The number of matches for RANSAC is not enough! Maybe the scan_line param is not right!" << endl;
            continue;
        }
        /*Remove the mismatches for aggregation keypoints*/
        vector<pair<int, int>> vTrueMatchedIndex;
        ransac(currentKeyPoints, lastKeyPoints, vMatched, vTrueMatchedIndex);  
        /*Remove the mismatches for edge keypoints*/     
        vector<pair<PointXYZSCA, PointXYZSCA>> vTrueMatchedEdgePoint;       
        ransacForEdgePt(matchedEdgePoint, vTrueMatchedEdgePoint);
                             
              
        /*Visualization*/
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = "map";
        line_list.header.stamp = ros::Time::now();
        line_list.scale.x = 0.07; 

        line_list.color.r = 0.0;
        line_list.color.g = 1.0;
        line_list.color.b = 0.3;
        line_list.color.a = 1.0;

        pcl::PointCloud<pcl::PointXYZ> allKeyPoints;        
        for(size_t i = 0; i < vTrueMatchedEdgePoint.size(); i++)
        {   
            PointXYZSCA pt1 = vTrueMatchedEdgePoint[i].first;
            PointXYZSCA pt2 = vTrueMatchedEdgePoint[i].second;      
                        
            Eigen::Vector3d pt2_(pt2.x, pt2.y, pt2.z);
            Eigen::Vector3d pt2_trans = T * pt2_;
            
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x = pt1.x;
            p1.y = pt1.y;
            p1.z = pt1.z;
            line_list.points.push_back(p1);

            p2.x = pt2_trans.x();
            p2.y = pt2_trans.y();
            p2.z = pt2_trans.z();
            line_list.points.push_back(p2);

            pcl::PointXYZ tmpPt1(pt1.x, pt1.y, pt1.z);
            pcl::PointXYZ tmpPt2(pt2_trans.x(), pt2_trans.y(), pt2_trans.z());

            allKeyPoints.points.push_back(tmpPt1);
            allKeyPoints.points.push_back(tmpPt2);
        }

        pubMarker.publish(line_list);

        sensor_msgs::PointCloud2 sensorKPt;
        pcl::toROSMsg(allKeyPoints, sensorKPt);
        sensorKPt.header.stamp = ros::Time::now();
        sensorKPt.header.frame_id = "map";
        pubKeyPoint.publish(sensorKPt);      


        sensor_msgs::PointCloud2 currentSensorCloud;
        pcl::toROSMsg(cur_cloud, currentSensorCloud);
        currentSensorCloud.header.stamp = ros::Time::now();
        currentSensorCloud.header.frame_id = "map";
        pubFullCloud1.publish(currentSensorCloud);           

        pcl::PointCloud<pcl::PointXYZ> last_transed_cloud;
        for(size_t i = 0; i < last_cloud.size(); i++)
        {
            pcl::PointXYZ pt = last_cloud.points[i];
            Eigen::Vector3d pt_(pt.x, pt.y, pt.z);
            Eigen::Vector3d pt_transed = T * pt_;
            pcl::PointXYZ pt_transed_(pt_transed.x(), pt_transed.y(), pt_transed.z());
            last_transed_cloud.points.push_back(pt_transed_);
        }

        sensor_msgs::PointCloud2 lastSensorCloud;
        pcl::toROSMsg(last_transed_cloud, lastSensorCloud);
        lastSensorCloud.header.stamp = ros::Time::now();
        lastSensorCloud.header.frame_id = "map";
        pubFullCloud2.publish(lastSensorCloud); 
                 
        ros::spinOnce();
        loop_rate.sleep();
    }
}
