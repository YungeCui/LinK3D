#include "LinK3D_extractor.h"

using namespace std;

namespace LinK3D_SLAM
{
    LinK3D_Extractor::LinK3D_Extractor(int nScans, float minimumRange, float XYDisTh, 
                        float xDisTh, float yDisTh, int ptNumTh, int scanNumTh, int scoreTh):
                        mNScans(nScans), mMinimumRange(minimumRange), mXYDisTh(XYDisTh),
                        mXDisTh(xDisTh), mYDisTh(yDisTh), mPtNumTh(ptNumTh), mScanNumTh(scanNumTh), mScoreTh(scoreTh){}

    void LinK3D_Extractor::removeClosedPoint(pcl::PointCloud<pcl::PointXYZ> &cloudIn, pcl::PointCloud<pcl::PointXYZ> &cloudOut)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloudIn, cloudIn, indices);
        if(&cloudIn != &cloudOut){
            cloudOut.header = cloudIn.header;
            cloudOut.points.resize(cloudIn.points.size());
        }

        size_t j = 0;

        for(size_t i = 0; i < cloudIn.points.size(); ++i)
        {
            if(cloudIn.points[i].x * cloudIn.points[i].x + cloudIn.points[i].y * cloudIn.points[i].y + cloudIn.points[i].z * cloudIn.points[i].z < mMinimumRange * mMinimumRange)
                continue;
            cloudOut.points[j] = cloudIn.points[i];
            j++;
        }

        if(j != cloudIn.points.size())
        {
            cloudOut.points.resize(j);
        }

        cloudOut.height = 1;
        cloudOut.width = static_cast<uint32_t>(j);
        cloudOut.is_dense = true;
    }

    void LinK3D_Extractor::getEdgePoint(pcl::PointCloud<pcl::PointXYZ> &laserCloudIn, MatPt &scanCloud)
    {
        std::vector<int> scanStartInd(mNScans, 0);
        std::vector<int> scanEndInd(mNScans, 0);       

        removeClosedPoint(laserCloudIn, laserCloudIn);

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;
    
        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }
        
        bool halfPassed = false;
        int count = cloudSize;
        pcl::PointXYZI point;
        std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(mNScans);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (mNScans == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (mNScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (mNScans == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (mNScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (mNScans == 64)
            {   
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = mNScans / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
            }
            
            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            { 
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            point.intensity = ori;
            laserCloudScans[scanID].points.push_back(point);
        }

        size_t scanSize = laserCloudScans.size();
        scanCloud.resize(scanSize);
        cloudSize = count;
        
        for(int i = 0; i < mNScans; i++)
        {
            int laserCloudScansSize = laserCloudScans[i].size();
            if(laserCloudScansSize >= 15)
            {
                for(int j = 5; j < laserCloudScansSize - 5; j++)
                {
                    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x
                                + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x
                                + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x
                                + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x
                                + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x
                                + laserCloudScans[i].points[j + 5].x;
                    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y
                                + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y
                                + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y
                                + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y
                                + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y
                                + laserCloudScans[i].points[j + 5].y;
                    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z
                                + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z
                                + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z
                                + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z
                                + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z
                                + laserCloudScans[i].points[j + 5].z;

                    float smoothness = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    if(smoothness > 10 && smoothness < 20000)  
                    {
                        float ori = laserCloudScans[i].points[j].intensity;
                        
                        PointXYZSCA tmpPt;
                        tmpPt.x = laserCloudScans[i].points[j].x;
                        tmpPt.y = laserCloudScans[i].points[j].y;
                        tmpPt.z = laserCloudScans[i].points[j].z;
                        tmpPt.scan = i;
                        tmpPt.smoothness = smoothness;
                        tmpPt.angel = ori; 
                        scanCloud[i].emplace_back(tmpPt);
                    }
                }
            }
        }
       
    }

    void LinK3D_Extractor::divideArea(MatPt &scanCloud, MatPt &areaCloud)
    {
        int scanSize = scanCloud.size();
        if(scanSize == 0){
            return;
        }

        areaCloud.resize(120);
        for(int i = 0; i < scanSize; i++) 
        {
            int pointSize = scanCloud[i].size();
            for(int j = 0; j < pointSize; j++)
            {                
                int areaNum = 0;
                float angel = scanCloud[i][j].angel;
                
                if(angel > 0 && angel < 2 * M_PI)
                {
                    areaNum = std::floor((angel / (2 * M_PI)) * 120);
                }   
                else if(angel > 2 * M_PI)
                {
                    areaNum = std::floor(((angel - 2 * M_PI) / (2 * M_PI)) * 120);
                }
                else if(angel < 0)
                {
                    areaNum = std::floor(((angel + 2 * M_PI) / (2 * M_PI)) * 120);
                }
                areaCloud[areaNum].push_back(scanCloud[i][j]);
            }
        }
    }

    float LinK3D_Extractor::computeClusterMeans(std::vector<PointXYZSCA> &oneCluster)
    {
        if(oneCluster.empty()){
            return 0;
        }

        float distSum = 0;
        int ptCnt = oneCluster.size();

        for(int i = 0; i < ptCnt; i++)
        {
            distSum += distXY(oneCluster[i]);
        }

        return (distSum/ptCnt);
    }

    void LinK3D_Extractor::computeXYMeans(std::vector<PointXYZSCA> &oneCluster, std::pair<float, float> &xyMeans)
    {
        if(oneCluster.empty()){
            return;
        }

        int ptCnt = oneCluster.size();
        float xSum = 0;
        float ySum = 0;

        for(int ptNum = 0; ptNum < ptCnt; ptNum++)
        {
            xSum += oneCluster[ptNum].x;
            ySum += oneCluster[ptNum].y;
        }

        float xMeans = xSum/ptCnt;
        float yMeans = ySum/ptCnt;
        xyMeans = std::make_pair(xMeans, yMeans);
    }

    void LinK3D_Extractor::computeCluster(const MatPt &areaCloud, MatPt &clustered)
    {    
        int areaSize = areaCloud.size();
        if(areaSize == 0){
            return;
        }

        MatPt tmpClustered;
        PointXYZSCA curvPt;
        std::vector<PointXYZSCA> dummy(1, curvPt); 

        //Cluster for each sector area
        for(int i = 0; i < areaSize; i++)
        {
            if(areaCloud[i].size() < 6)  
                continue;

            int ptCnt = areaCloud[i].size();        
            MatPt curAreaCluster(1, dummy);
            curAreaCluster[0][0] = areaCloud[i][0];

            for(int j = 1; j < ptCnt; j++)
            {
                int clusterCnt = curAreaCluster.size();

                for(int k = 0; k < clusterCnt; k++)
                {
                    float means = computeClusterMeans(curAreaCluster[k]);
                    std::pair<float, float> xyMeans;
                    computeXYMeans(curAreaCluster[k], xyMeans);
                    
                    PointXYZSCA tmpPt = areaCloud[i][j];
              
                    if(abs(distXY(tmpPt) - means) < mXYDisTh && abs(xyMeans.first - tmpPt.x) < mXDisTh && abs(xyMeans.second - tmpPt.y) < mYDisTh){
                        curAreaCluster[k].emplace_back(tmpPt);
                        break;
                    }else if(abs(distXY(tmpPt) - means) >= mXYDisTh && k == clusterCnt-1){
                        curAreaCluster.emplace_back(dummy);
                        curAreaCluster[clusterCnt][0] = tmpPt;
                    }else{ 
                        continue; 
                    }
                    
                }
            }

            int clusterCnt = curAreaCluster.size();
            for(int clusterNum = 0; clusterNum < clusterCnt; clusterNum++)
            {
                int ptCnt = curAreaCluster[clusterNum].size();
                if(ptCnt < 10){
                    continue;
                }
                tmpClustered.emplace_back(curAreaCluster[clusterNum]);
            }

        }

        //Merge the adjacent clusters
        int clusterCnt = tmpClustered.size();
        
        std::vector<bool> toBeMerge(clusterCnt, false);
        std::multimap<int, int> mToBeMergeInd;
        std::set<int> sNeedMergeInd;

        for(int i = 0; i < clusterCnt; i++)
        {
            if(toBeMerge[i]){
                continue;
            }
            float means1 = computeClusterMeans(tmpClustered[i]);
            std::pair<float, float> xyMeans1;
            computeXYMeans(tmpClustered[i], xyMeans1);

            for(int j = 1; j < clusterCnt; j++)
            {
                if(toBeMerge[j]){
                    continue;
                }
                float means2 = computeClusterMeans(tmpClustered[j]);
                std::pair<float, float> xyMeans2;
                computeXYMeans(tmpClustered[j], xyMeans2);

                if(abs(means1 - means2) < 0.6 && abs(xyMeans1.first - xyMeans2.first) < 0.8 && abs(xyMeans1.second - xyMeans2.second) < 0.8) 
                {
                    mToBeMergeInd.insert(std::make_pair(i, j));
                    sNeedMergeInd.insert(i);
                    toBeMerge[i] = true;
                    toBeMerge[j] = true;
                }
            }

        }

        if(sNeedMergeInd.empty()){
            for(int i = 0; i < clusterCnt; i++)
            {
                clustered.emplace_back(tmpClustered[i]);
            }
        }
        else
        {
            for(int i = 0; i < clusterCnt; i++)
            {
                if(toBeMerge[i] == false)
                {
                    clustered.emplace_back(tmpClustered[i]);
                }
            }
            
            for(auto setIt = sNeedMergeInd.begin(); setIt != sNeedMergeInd.end(); ++setIt)
            {
                int needMergeInd = *setIt;
                auto entries = mToBeMergeInd.count(needMergeInd);
                auto iter = mToBeMergeInd.find(needMergeInd);
                std::vector<int> vInd;
                while(entries){
                    int ind = iter->second;
                    vInd.emplace_back(ind);
                    ++iter;
                    --entries;
                }
                clustered.emplace_back(tmpClustered[needMergeInd]);
                size_t cnt = clustered.size();
                for(size_t j = 0; j < vInd.size(); j++)
                {
                    for(size_t ptNum = 0; ptNum < tmpClustered[vInd[j]].size(); ptNum++)
                    {
                        clustered[cnt - 1].emplace_back(tmpClustered[vInd[j]][ptNum]);
                    }
                }
            }
        }       

    }

    void LinK3D_Extractor::computeDirection(pcl::PointXYZ ptFrom, pcl::PointXYZ ptTo, Eigen::Vector2f &direction)
    {
        direction(0, 0) = ptTo.x - ptFrom.x;
        direction(1, 0) = ptTo.y - ptFrom.y;
    }

    std::vector<pcl::PointXYZ> LinK3D_Extractor::getAggregationKeyPt(const MatPt &clustered, std::vector<int> &index)
    {
        std::vector<pcl::PointXYZ> keyPoints;
        int clusterCnt = clustered.size();
        for(int clusterNum = 0; clusterNum < clusterCnt; clusterNum++)
        {
            int ptCnt = clustered[clusterNum].size();  
   
            if(ptCnt < mPtNumTh){
                continue;
            }

            std::vector<PointXYZSCA> tmpCluster;
            std::set<int> scans;
            float x = 0, y = 0, z = 0;
            for(int ptNum = 0; ptNum < ptCnt; ptNum++)
            {
                PointXYZSCA pt = clustered[clusterNum][ptNum];          
                int scan = pt.scan;
                scans.insert(scan);

                x += pt.x;
                y += pt.y;
                z += pt.z;
            }

            if((int)scans.size() < mScanNumTh){
                continue;
            }

            pcl::PointXYZ pt(x/ptCnt, y/ptCnt, z/ptCnt);
            keyPoints.emplace_back(pt);
            index.emplace_back(clusterNum); 
        }
        return keyPoints;
    }


    void LinK3D_Extractor::getDescriptor(const std::vector<pcl::PointXYZ> &keyPoints, cv::Mat &descriptors)
    {
        if(keyPoints.size() < 5){
            cout << "The number of keypoints is too small!" << endl;
            return;
        }

        size_t ptSize = keyPoints.size();

        descriptors = cv::Mat::zeros(ptSize, 180, CV_32FC1); 

        vector<vector<float>> disTab;
        vector<float> oneRowDis(ptSize, 0);
        disTab.resize(ptSize, oneRowDis);

        vector<vector<Eigen::Vector2f>> directionTab;
        Eigen::Vector2f direct(0, 0);
        vector<Eigen::Vector2f> oneRowDirect(ptSize, direct);
        directionTab.resize(ptSize, oneRowDirect);

        for(size_t i = 0; i < keyPoints.size(); i++)
        {
            for(size_t j = i+1; j < keyPoints.size(); j++)
            {
                float dist = distPt2Pt(keyPoints[i], keyPoints[j]);
                disTab[i][j] = fRound(dist);
                disTab[j][i] = disTab[i][j];

                Eigen::Vector2f tmpDirection;
                                
                tmpDirection(0, 0) = keyPoints[j].x - keyPoints[i].x;
                tmpDirection(1, 0) = keyPoints[j].y - keyPoints[i].y;

                directionTab[i][j] = tmpDirection;
                directionTab[j][i] = -tmpDirection;
            }
        }

        
        for(size_t i = 0; i < keyPoints.size(); i++)
        {
            vector<float> tempRow(disTab[i]);
            std::sort(tempRow.begin(), tempRow.end());

            //The number of the used closest keypoints is set to 3
            int Index[3];  
            for(int k = 0; k < 3; k++)  
            {
                vector<float>::iterator it1 = find(disTab[i].begin(), disTab[i].end(), tempRow[k+1]);
                if(it1 == disTab[i].end()){
                    continue;
                }else{
                    Index[k] = std::distance(disTab[i].begin(), it1);
                }
            }

            for(int indNum = 0; indNum < 3; indNum++) 
            {
                size_t index = Index[indNum];
                Eigen::Vector2f mainDirect;
                
                mainDirect = directionTab[i][index];

                //180-dimensional descriptor corresponds to 180 sector areas.
                vector<vector<float>> areaDis(180);  
                areaDis[0].emplace_back(disTab[i][index]);
                
                for(size_t j = 0; j < keyPoints.size(); j++)
                {
                    if(j == i || j == index){
                        continue;
                    }
                    
                    Eigen::Vector2f otherDirect = directionTab[i][j];
                
                    Eigen::Matrix2f matrixDirect;
                    matrixDirect << mainDirect(0, 0) , mainDirect(1, 0) , otherDirect(0, 0) , otherDirect(1, 0);
                    float deter = matrixDirect.determinant();

                    int areaNum = 0;
                    double cosAng = (double)mainDirect.dot(otherDirect) / (double)(mainDirect.norm() * otherDirect.norm());                                 
                    if(abs(cosAng) - 1 > 0){   
                        continue;
                    }
                    
                    float angel = acos(cosAng) * 180 / M_PI;
                    
                    if(angel < 0 || angel > 180){
                        continue;
                    }
                    
                    if(deter > 0){
                        areaNum = ceil((angel - 1) / 2); 
                        
                    }else{
                        if(angel - 2 < 0){ 
                            areaNum = 0;
                        }else{
                            angel = 360 - angel;
                            areaNum = ceil((angel - 1) / 2); 
                        }   
                    }
                    if(areaNum != 0){
                        areaDis[areaNum].emplace_back(disTab[i][j]);
                    }
                            
                }
                
                //Each dimension of the descriptor corresponds to the distance between the current keypoint and the closest keypoint in the corresponding sector area.
                float *descriptor = descriptors.ptr<float>(i);
                for(int areaNum = 0; areaNum < 180; areaNum++) 
                {
                    if(areaDis[areaNum].size() == 0){
                        continue;
                    }else{
                        std::sort(areaDis[areaNum].begin(), areaDis[areaNum].end());
                        if(descriptor[areaNum] == 0)
                        {
                            descriptor[areaNum] = areaDis[areaNum][0]; 
                        }                        
                    }
                }                
            }
        }        
    }

    float LinK3D_Extractor::fRound(float in)
    {
        float f;
        int temp = std::round(in * 10);
        f = temp/10.0;
        
        return f;
    }

    //The entrance of extracting LinK3D features
    void LinK3D_Extractor::operator()(pcl::PointCloud<pcl::PointXYZ> &cloudIn, std::vector<pcl::PointXYZ> &keyPoints, cv::Mat &descriptors, std::vector<int> &index, MatPt &clustered)
    {
        if(cloudIn.empty()){
            return;
        }         

        MatPt scanCloud;
        getEdgePoint(cloudIn, scanCloud);
        
        MatPt areaCloud;
        divideArea(scanCloud, areaCloud);       
               
        computeCluster(areaCloud, clustered);
        
        keyPoints = getAggregationKeyPt(clustered, index);
        
        getDescriptor(keyPoints, descriptors);         
    }

    void LinK3D_Extractor::matcher(cv::Mat &descriptors1, cv::Mat &descriptors2, vector<pair<int, int>> &vMatched)
    {
        int ptSize1 = descriptors1.rows;
        int ptSize2 = descriptors2.rows;
        
        std::multimap<int, int> matchedIndexScore;      
        std::multimap<int, int> mMatchedIndex;
        set<int> sIndex;
        
        for(int i = 0; i < ptSize1; i++)
        {
            std::pair<int, int> highestIndexScore(0, 0);
            float* pDes1 = descriptors1.ptr<float>(i);        

            for(int j = 0; j < ptSize2; j++)
            {
                int sameBitScore = 0;
                float* pDes2 = descriptors2.ptr<float>(j);
                
                for(int bitNum = 0; bitNum < 180; bitNum++)
                {
                    if(pDes1[bitNum] != 0 && pDes2[bitNum] != 0 && abs(pDes1[bitNum] - pDes2[bitNum]) <= 0.2){
                        sameBitScore += 1;
                    }             
                }

                if(sameBitScore > highestIndexScore.second){
                    highestIndexScore.first = j;
                    highestIndexScore.second = sameBitScore;
                }

            }
            //According to i, get the score
            matchedIndexScore.insert(std::make_pair(i, highestIndexScore.second)); 
            //According to j, get i
            mMatchedIndex.insert(std::make_pair(highestIndexScore.first, i)); 
            sIndex.insert(highestIndexScore.first);
        }
        
        //Remove one-to-multiple matches for descriptor2
        for(std::set<int>::iterator setIt = sIndex.begin(); setIt != sIndex.end(); ++setIt)
        {
            int indexJ = *setIt;
            auto entries = mMatchedIndex.count(indexJ);

            if(entries == 1){
                auto iterI = mMatchedIndex.find(indexJ);
                auto iterScore = matchedIndexScore.find(iterI->second);
                if(iterScore->second >= mScoreTh){   
                    vMatched.emplace_back(std::make_pair(iterI->second, indexJ));
                }           
            }else{ 
                auto iter1 = mMatchedIndex.find(indexJ);
                int highestScore = 0;
                int highestScoreIndex = -1;

                while(entries){
                    int indexI = iter1->second;
                    auto iterScore = matchedIndexScore.find(indexI);
                    if(iterScore->second > highestScore){
                        highestScore = iterScore->second;
                        highestScoreIndex = indexI;
                    }                
                    ++iter1;
                    --entries;
                }
                if(highestScore >= mScoreTh){   
                    vMatched.emplace_back(std::make_pair(highestScoreIndex, indexJ));
                }            
            }
            
        }
    }

    //Get the true edge keypoints with higher smoothness
    void LinK3D_Extractor::filterLowSmooth(MatPt &clustered, MatPt &filtered)
    {
        if(clustered.empty()){
            return;
        }

        int clusterSize = clustered.size();

        filtered.resize(clusterSize);
        for(int i = 0; i < clusterSize; i++)
        {
            int ptCnt = clustered[i].size();
            MatPt tmpCluster;
            vector<int> vScanID;
            for(int j = 0; j < ptCnt; j++)
            {
                PointXYZSCA pt = clustered[i][j];
                int scan = pt.scan;
                auto it = std::find(vScanID.begin(), vScanID.end(), scan);
                if(it == vScanID.end()){
                    vScanID.emplace_back(scan);
                    vector<PointXYZSCA> vPt(1, pt);
                    tmpCluster.emplace_back(vPt);
                }else{
                    int filteredInd = std::distance(vScanID.begin(), it);
                    tmpCluster[filteredInd].emplace_back(pt);
                }
            }

            for(size_t scanCnt = 0; scanCnt < tmpCluster.size(); scanCnt++)
            {
                if(tmpCluster[scanCnt].size() == 1){
                    filtered[i].emplace_back(tmpCluster[scanCnt][0]);
                }
                else
                {
                    float maxSmooth = 0;
                    PointXYZSCA maxSmoothPt;
                    for(size_t j = 0; j < tmpCluster[scanCnt].size(); j++)
                    {
                        if(tmpCluster[scanCnt][j].smoothness > maxSmooth){
                            maxSmooth = tmpCluster[scanCnt][j].smoothness;
                            maxSmoothPt = tmpCluster[scanCnt][j];
                        }
                    }
                    filtered[i].emplace_back(maxSmoothPt);
                }
            }  
        }
    }

    void LinK3D_Extractor::matchEdgePoints(MatPt &filtered1, MatPt &filtered2, vector<int> &index1, vector<int> &index2, MatPt &matchedPoint, vector<pair<int, int>> &vMatched)
    {
        if(vMatched.empty()){
            return;
        }

        for(size_t i = 0; i < vMatched.size(); i++)
        {
            pair<int, int> matchedInd = vMatched[i];
            int ind1 = index1[matchedInd.first];
            int ind2 = index2[matchedInd.second];
            
            int ptSize1 = filtered1[ind1].size();
            int ptSize2 = filtered2[ind2].size();

            std::map<int, int> mScanID_Ind1;
            std::map<int, int> mScanID_Ind2;

            for(int ptNum1 = 0; ptNum1 < ptSize1; ptNum1++)
            {
                int scanID1 = filtered1[ind1][ptNum1].scan;
                pair<int, int> scanID_Ind(scanID1, ptNum1);
                mScanID_Ind1.insert(scanID_Ind);
            }

            for(int ptNum2 = 0; ptNum2 < ptSize2; ptNum2++)
            {
                int scanID2 = filtered2[ind2][ptNum2].scan;
                pair<int, int> scanID_Ind(scanID2, ptNum2);
                mScanID_Ind2.insert(scanID_Ind);
            }

            for(auto it1 = mScanID_Ind1.begin(); it1 != mScanID_Ind1.end(); it1++)
            {
                int scanID1 = (*it1).first;
                auto it2 = mScanID_Ind2.find(scanID1);
                if(it2 == mScanID_Ind2.end()){
                    continue;
                }
                else
                {
                    vector<PointXYZSCA> tmpMatchPt;
                    PointXYZSCA pt1 = filtered1[ind1][(*it1).second];
                    PointXYZSCA pt2 = filtered2[ind2][(*it2).second];
                    tmpMatchPt.emplace_back(pt1);
                    tmpMatchPt.emplace_back(pt2);
                    matchedPoint.emplace_back(tmpMatchPt);
                }
            }
        }
    }

}
