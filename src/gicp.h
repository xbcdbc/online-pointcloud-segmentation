#ifndef _G_ICP_H
#define _G_ICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

typedef Matrix<double, 6, 1> Vector3d pos_type;

class Generalized_ICP
{
    public:
    Generalized_ICP(int space_dimension,double max_dist):
    _space_dimension(space_dimension),
    _max_dist(max_dist)
    {
        _initial_pos<<0,0,0,0,0,0;
        _final_pos<<0,0,0,0,0,0;
    }
    ~Generalized_ICP();

    private:
    int _space_dimension;    //0:3D, 1:2D ,other:error
    double _max_dist;
    Eigen::VectorXd _initial_pos;
    Eigen::VectorXd _final_pos;

    pcl::PointCloud<pcl::PointXYZI>::Ptr  _reference;
    pcl::PointCloud<pcl::PointXYZI>::Ptr  _current;

    void set_max_dist(double max_dist)
    {
        _max_dist=max_dist;
    }
    void set_initial_pos(pos_type initial_pos)
    {
        _initial_pos=initial_pos;
    }
    void set_reference_points(pcl::PointCloud<pcl::PointXYZI>::Ptr reference);
    void set_final_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input current);

    void register_pcl();
    void register_pcl_byline();
};

#endfi