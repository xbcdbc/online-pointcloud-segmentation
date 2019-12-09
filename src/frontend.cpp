#include <pcl/kdtree/kdtree_flann.h>
#include "frontend.h"
#include <math.h>
#include <vector>

using namespace std;

Generalized_ICP::~Generalized_ICP
{

}

void Generalized_ICP::set_reference_points(pcl::PointCloud<pcl::point_type>::Ptr reference)
{
    _reference=reference;
}
void Generalized_ICP::set_final_points(pcl::PointCloud<pcl::point_type>::Ptr input current)
{
    _current=current;
}
point_type trans_point(point_type point,pos_type T)
{
    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
 
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=yawAngle*pitchAngle*rollAngle;

    Eigen::Vector3d eigen_point,result_point;
    eigen_point<<point.x,point.y,point.z;
    result_point=rotation_matrix*eigen_point+T.header(3);
    
    point_type trans_point;
    trans_point.x=result_point[0];
    trans_point.y=result_point[1];
    trans_point.z=result_point[2];
    return trans_point;
}
void Generalized_ICP::register_pcl_by_GICP()
{
    double max_dist2=pow(_max_dist,2);
    int num_matches=0;
    int num_points=_current->points.size();
    pos_type last_pos;
    last_pos<<0,0,0,0,0,0;

    if(num_points==0){

    }
    //construct kd-tree
    pcl::KdTreeFLANN<point_type> kdtree;
    kdtree.setInputCloud (_reference);
    //find correspondence points
    int K=1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(int i=0;i<num_points;i++)
    {
        point_type searchPoint=trans_point(_current->points[i],_initial_pos);
        if(kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if(pointNKNSquaredDistance[0]<_max_dist)
            {

            }
            else
            {
                //not match
            }
            
        }
    }
    //optimize
}


/*find correspondence by scan line*/
void Generalized_ICP::register_pcl_by_loam()
{
    float horizon_resolution=0.478/ 180.0 * M_PI;
    float vertical_resolution=0.495/ 180.0 * M_PI;
    float cloudCurvature[N_SCAN*Horizon_SCAN];
    int cloudNeighborPicked[N_SCAN*Horizon_SCAN];
    int cloudLabel[N_SCAN*Horizon_SCAN];
    struct smoothness_t{ 
        float value;
        size_t ind;
    };
    std::vector<smoothness_t> cloudSmoothness;
    //calculateSmoothness/curvature
    int cloudSize = _current_info->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {

        float diffRange = _current_info->points[i-5].intensity + _current_info->points[i-4].intensity
                        + _current_info->points[i-3].intensity + _current_info->points[i-2].intensity
                        + _current_info->points[i-1].intensity - _current_info->points[i].intensity * 10
                        + _current_info->points[i+1].intensity + _current_info->points[i+2].intensity
                        + _current_info->points[i+3].intensity+ _current_info->points[i+4].intensity
                        + _current_info->points[i+5].intensity;            

        cloudCurvature[i] = diffRange*diffRange;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;

        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
    //mark occludepoints
    for (int i = 5; i < cloudSize - 6; ++i){

        float depth1 = _current_info->points[i].intensity;
        float depth2 = _current_info->points[i+1].intensity;

        float yaw1= atan2(_current_info->points[i].y,_current_info->points[i].x);
        float yaw2= atan2(_current_info->points[i+1].y,_current_info->points[i+1].y);
        float yawDiff = fabs(yaw1-yaw2);

        if (yawDiff < 10*horizon_resolution){

            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(_current_info->points[i-1].intensity - _current_info->points[i].intensity));
        float diff2 = std::abs(float(_current_info->points[i+1].intensity - _current_info->points[i].intensity));

        if (diff1 > 0.02 * _current_info->points[i].intensity && diff2 > 0.02 * _current_info->points[i].intensity)
            cloudNeighborPicked[i] = 1;


        //extract features
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        for (int i = 0; i < N_SCAN; i++) {

            surfPointsLessFlatScan->clear();

            for (int j = 0; j < 6; j++) {

                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == false) {
                    
                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == true) {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(segmentedCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }
    }
}