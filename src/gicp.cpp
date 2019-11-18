#include "gicp.h"

Generalized_ICP::~Generalized_ICP
{

}

void Generalized_ICP::set_reference_points(pcl::PointCloud<pcl::PointXYZI>::Ptr reference)
{
    _reference=reference;
}
void Generalized_ICP::set_final_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input current)
{
    _current=current;
}

void Generalized_ICP::register_pcl()
{

}


/*find correspondence by scan line*/
void Generalized_ICP::register_pcl_byline()
{

}