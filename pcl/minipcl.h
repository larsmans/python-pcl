#ifndef _MINIPCL_H_
#define _MINIPCL_H_

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

void mpcl_compute_normals(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                          int ksearch,
                          double searchRadius,
                          pcl::PointCloud<pcl::Normal> &out);

void mpcl_estimate_boundaries(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                              float angle_threshold,
                              double search_radius,
                              int normal_ksearch,
                              double normal_search_radius,
                              char *out, size_t n);

void mpcl_sacnormal_set_axis(pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> &sac,
                             double ax, double ay, double az);

void mpcl_extract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &incloud,
                  pcl::PointCloud<pcl::PointXYZRGB> *outcloud,
                  pcl::PointIndices *indices,
                  bool negative);

#endif
