/*
 * @Descripttion: 
 * @version: 
 * @Author: Peng Wei
 * @Email: poetpw@163.com
 * @Date: 2019-11-07 14:46:42
 * @LastEditors: Peng Wei
 * @LastEditTime: 2019-12-09 11:47:56
 */

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>

typedef pcl::PointXYZI  PointType;

class RIBSegment{
private:
	pcl::PointCloud<PointType>::Ptr laserCloudIn;
    cv::Mat rangeMat; // range matrix for range image
    uint16_t groundScanInd;
    int Horizon_SCAN;
    int N_SCAN;
    double sensorMountAngle;
    uint16_t segmentValidPointNum;
    int segmentValidLineNum;
    double segmentAlphaX;
    double segmentAlphaY;
    double segmentTheta;

    pcl::PointCloud<PointType>::Ptr fullCloud;     // projected velodyne raw cloud, but saved in the form of 1-D matrix
   
    PointType nanPoint; // fill in fullCloud at each iteration
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;     // array for breadth-first search process of segmentation
    uint16_t *queueIndY;

    
public:
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr NonegroundCloud;
    pcl::PointCloud<PointType>::Ptr groundCloud;

    cv::Mat labelMat; // label matrix for segmentaiton marking

	RIBSegment();
	void allocateMemory();

    void resetParameters(pcl::PointCloud<PointType>::Ptr input);
    ~RIBSegment();

    void groundRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);
};
