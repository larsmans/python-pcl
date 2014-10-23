#ifndef _REGISTRATION_HELPER_H_
#define _REGISTRATION_HELPER_H_

#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>

void mpcl_sac_ia_init(pcl::PointCloud<pcl::PointXYZRGB> &,
                      pcl::PointCloud<pcl::PointXYZRGB> &,
                      double, double, double,
                      pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB,
                                                           pcl::PointXYZRGB,
                                                           pcl::FPFHSignature33>&);

#endif
