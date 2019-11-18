
#include "ros/ros.h"
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv/cv.h>

#include "RIBSegment.h"

pcl::PointCloud<pcl::PointXYZI> data;
ros::Publisher pub_segment;
ros::Publisher pub_none_ground;

RIBSegment segment;

void callbackPointclouds(const sensor_msgs::PointCloud2ConstPtr& input)
{
    std::cout<<"new data come"<<std::endl;
    pcl::fromROSMsg(*input,data);
    std::cout<<"points size:"<<data.points.size()<<std::endl;
    segment.resetParameters(data.makeShared());
    segment.groundRemoval();
    std::cout<<"remove ground"<<std::endl;
    segment.cloudSegmentation();
    std::cout<<"segment size:"<<segment.segmentedCloud->points.size()<<std::endl;

    sensor_msgs::PointCloud2 points_seg_msg,NoneGround_msg;
    pcl::toROSMsg(*(segment.segmentedCloud),points_seg_msg);
    pcl::toROSMsg(*(segment.NonegroundCloud),NoneGround_msg);

    points_seg_msg.header=input->header;
    NoneGround_msg.header=input->header;
    pub_segment.publish(points_seg_msg);
    pub_none_ground.publish(NoneGround_msg);
    std::cout<<"process a frame data"<<std::endl<<std::endl;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_segment");

    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud;
    
    std::string pointcloud_name="/royale_camera_driver/point_cloud_sub";


    sub_pointcloud = nh.subscribe (pointcloud_name.c_str(), 5, &callbackPointclouds);
    pub_segment = nh.advertise<sensor_msgs::PointCloud2> ("point_segment", 1);
    pub_none_ground= nh.advertise<sensor_msgs::PointCloud2> ("point_none_ground", 1);

    ros::spin();
}


 