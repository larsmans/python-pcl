#ifndef _REGISTRATION_HELPER_H_
#define _REGISTRATION_HELPER_H_

#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>

void mpcl_sac_ia_init(pcl::PointCloud<pcl::PointXYZRGBNormal> &,
                      pcl::PointCloud<pcl::PointXYZRGBNormal> &,
                      double, double, double,
                      pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBNormal,
                                                           pcl::PointXYZRGBNormal,
                                                           pcl::FPFHSignature33>&);

#endif
