/*
 * @Descripttion: 
 * @version: 
 * @Author: Peng Wei
 * @Email: poetpw@163.com
 * @Date: 2019-11-07 14:46:42
 * @LastEditors: Peng Wei
 * @LastEditTime: 2019-12-09 11:47:56
 */

#include "RIBSegment.h"
#include <opencv2/opencv.hpp>


RIBSegment::RIBSegment():
groundScanInd(20),
Horizon_SCAN(224),
N_SCAN(38),
sensorMountAngle(0.0),
segmentValidPointNum(10),
segmentValidLineNum(5),
segmentAlphaX(0.478/ 180.0 * M_PI),
segmentAlphaY(0.495/ 180.0 * M_PI),
segmentTheta(45/180.0 * M_PI)
{
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
}


void RIBSegment::allocateMemory(){

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    fullCloud.reset(new pcl::PointCloud<PointType>());

    groundCloud.reset(new pcl::PointCloud<PointType>());
    NonegroundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
   
    fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
}

void RIBSegment::resetParameters(pcl::PointCloud<PointType>::Ptr input){
    
    laserCloudIn=input;
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundCloud->clear();
    NonegroundCloud->clear();
    segmentedCloud->clear();

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    size_t rowIdn=0,columnIdn=0,index=0;
    double range=0.0;
    PointType thisPoint;
    size_t cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        
        rowIdn = i/Horizon_SCAN;
        columnIdn = i%Horizon_SCAN;
    
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < 0.07)
        {
            continue;
        }
        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
}

RIBSegment::~RIBSegment(){}

void RIBSegment::groundRemoval(){
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (int j = Horizon_SCAN-1; j >=0; --j){
        for (int i = N_SCAN-1; i >=groundScanInd ; --i){
            lowerInd = j + ( i )*Horizon_SCAN;
            upperInd = j + (i-1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1){
                // no info to check, invalid points
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }
                
            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (fabs(angle - sensorMountAngle) <= 25){
                groundMat.at<int8_t>(i,j) = 1;
                groundMat.at<int8_t>(i-1,j) = 1;
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                labelMat.at<int>(i,j) = -1;
            }

            if( groundMat.at<int8_t>(i,j) ==0  && rangeMat.at<float>(i,j) != FLT_MAX)
            {
                NonegroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);           
            }  
        }
    }
    
    for (size_t i = N_SCAN-1; i >=groundScanInd; i--){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == 1)
            {
                groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
            }   
        }
    }
}

void RIBSegment::cloudSegmentation(){
    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
            if (labelMat.at<int>(i,j) == 0)
            {
                labelComponents(i, j);
            }
        }
    std::cout<<"segment complete:"<<labelCount<<std::endl;
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){ 
            if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                segmentedCloud->points.back().intensity = labelMat.at<int>(i,j);
            }
        }
    }
}

void RIBSegment::labelComponents(int row, int col){
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; 
    bool lineCountFlag[N_SCAN] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    
    while(queueSize > 0){
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            //std::cout<<angle/M_PI*180<<std::endl;
            if (angle > segmentTheta){

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;            
    }
    // segment is valid, mark these points
    if (feasibleSegment == true){
        ++labelCount;
    }else{ // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i){
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

