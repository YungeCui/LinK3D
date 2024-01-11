#include <vector>
#include <map>
#include <set>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>

using namespace std;

struct PointXYZSCA
    {
        PCL_ADD_POINT4D;
        int scan;
        float smothness;
        float angel;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    }EIGEN_ALIGN16;

#define PC_SCA pcl::PointCloud<PointXYZSCA>

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZSCA, 
        (float, x, x)(float, y, y)(float, z, z)(int, scan, scan)(float, smothness, smothness)(float, angel, angel))

namespace LinK3D_SLAM
{
    #define distXY(a) sqrt(a.x * a.x + a.y * a.y)
    #define distPt2Pt(a, b) sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z))
    
    typedef std::vector<std::vector<PointXYZSCA>> MatPt;

    class LinK3D_Extractor
    {
        public:
            LinK3D_Extractor(int nScans, float minimumRange, float disTh, float XYDisTh, 
                              float yDisTh, int ptNumTh, int scanNumTh, int scoreTh);

            ~LinK3D_Extractor(){}

            void operator()(pcl::PointCloud<pcl::PointXYZ> &cloudIn, std::vector<pcl::PointXYZ> &vKeyPoint, cv::Mat &descriptors, std::vector<int> &index, MatPt &clustered);

            void removeClosedPoint(pcl::PointCloud<pcl::PointXYZ> &cloudIn, pcl::PointCloud<pcl::PointXYZ> &cloudOut);

            void getEdgePoint(pcl::PointCloud<pcl::PointXYZ> &laserCloudIn, MatPt &scanCloud);

            void divideArea(MatPt &matPt, MatPt &areaCloud);

            float computeClusterMeans(std::vector<PointXYZSCA> &oneCluster);

            void computeXYMeans(std::vector<PointXYZSCA> &oneCluster, std::pair<float, float> &xyMeans);

            void computeCluster(const MatPt &areaCloud, MatPt &clustered);

            void computeDirection(pcl::PointXYZ ptFrom, pcl::PointXYZ ptTo, Eigen::Vector2f &direction);

            std::vector<pcl::PointXYZ> getAggregationKeyPt(const MatPt &clustered, std::vector<int> &index);
            
            void getDescriptor(const std::vector<pcl::PointXYZ> &keyPoints, cv::Mat &descriptors);
            
            float fRound(float in);

            void matcher(cv::Mat &descriptors1, cv::Mat &descriptors2, vector<pair<int, int>> &vMatched);

            void filterLowSmooth(MatPt &clustered, MatPt &filtered);

            void matchEdgePoints(MatPt &filtered1, MatPt &filtered2, vector<int> &index1, vector<int> &index2, MatPt &matchedPoint, vector<pair<int, int>> &vMatched);
            
        private:
            int mNScans;

            float mMinimumRange;
            float mXYDisTh;
            float mXDisTh;
            float mYDisTh;

            int mPtNumTh;
            int mScanNumTh;

            int mScoreTh;
    };
}