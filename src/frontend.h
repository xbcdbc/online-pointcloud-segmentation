#ifndef _FRONTEND_H
#define _FRONTEND_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

typedef Matrix<double, 6, 1> Vector3d pos_type;
typedef pcl::PointXYZ  point_type;

class Generalized_ICP
{
    public:
    Generalized_ICP(int space_dimension,double max_dist):
    _space_dimension(space_dimension),
    _max_dist(max_dist),
    _MaxIterationInner(8),
    _MaxIteration(100)
    {
        _initial_pos<<0,0,0,0,0,0;
        _final_pos<<0,0,0,0,0,0;
        _reference.reset(new pcl::PointCloud<point_type>());
        _current.reset(new pcl::PointCloud<point_type>());
        _current_info.reset(new pcl::PointCloud<point_type>());
    }
    ~Generalized_ICP();

    private:
    int _space_dimension;          //0:3D, 1:2D ,other:error
    double _max_dist;
    int _MaxIterationInner;
    int _MaxIteration;

    Eigen::VectorXd _initial_pos;  //x,y,z,yaw,pitch,roll
    Eigen::VectorXd _final_pos;    //x,y,z,yaw,pitch,roll

    pcl::PointCloud<pcl::point_type>::Ptr  _reference;
    pcl::PointCloud<pcl::point_type>::Ptr  _current;
    pcl::PointCloud<pcl::point_type>::Ptr  _current_info;

    void set_max_dist(double max_dist)
    {
        _max_dist=max_dist;
    }
    void set_initial_pos(pos_type initial_pos)
    {
        _initial_pos=initial_pos;
    }
    void set_reference_points(pcl::PointCloud<pcl::point_type>::Ptr reference);
    void set_final_points(pcl::PointCloud<pcl::point_type>::Ptr input current);

    point_type trans_point(point_type point,pos_type T);

    void register_pcl_by_GICP();
    void register_pcl_by_loam();

    
};

#endfi