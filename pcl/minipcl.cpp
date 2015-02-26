#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>

#include "minipcl.h"

// set ksearch and radius to < 0 to disable 
void mpcl_compute_normals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
                          int ksearch,
                          double searchRadius,
                          pcl::PointCloud<pcl::Normal> &out)
{
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud.makeShared());
    if (ksearch >= 0)
        ne.setKSearch (ksearch);
    if (searchRadius >= 0.0)
        ne.setRadiusSearch (searchRadius);
    ne.compute (out);
}

// Boundary estimation (uses normal estimation, hence the parameters).
void mpcl_estimate_boundaries(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
                              float angle_threshold,
                              double search_radius,
                              int normal_ksearch,
                              double normal_search_radius,
                              char *out, size_t n)
{
    pcl::PointCloud<pcl::Normal> normals;
    mpcl_compute_normals(cloud, normal_ksearch, normal_search_radius, normals);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud(cloud.makeShared());
    est.setInputNormals(normals.makeShared());

    if (search_radius >= 0) {
        est.setRadiusSearch(search_radius);
    }

    est.setSearchMethod(typename pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
                            new pcl::search::KdTree<pcl::PointXYZRGBNormal>));
    est.compute(boundaries);

    for (size_t i = 0; i < n; i++) {
        out[i] = boundaries[i].boundary_point;
    }
}

void mpcl_sacnormal_set_axis(pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::Normal> &sac,
                             double ax, double ay, double az)
{
    Eigen::Vector3f vect(ax,ay,az);
    sac.setAxis(vect);
}

void mpcl_extract(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &incloud,
                  pcl::PointCloud<pcl::PointXYZRGBNormal> *outcloud,
                  pcl::PointIndices *indices,
                  bool negative)
{
    pcl::PointIndices::Ptr indicesptr (indices);
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> ext;
    ext.setInputCloud(incloud);
    ext.setIndices(indicesptr);
    ext.setNegative(negative);
    ext.filter(*outcloud);
}
