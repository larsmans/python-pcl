#ifndef _REGISTRATION_HELPER_H_
#define _REGISTRATION_HELPER_H_

#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>

void mpcl_sample_consensus_initial_alignment_init(pcl::PointCloud<pcl::PointXYZRGB> &cloud_source_ptr,
                          pcl::PointCloud<pcl::PointXYZRGB> &cloud_target_ptr,
                          double searchRadius, double minSampleDistance, double maxCorrespondenceDistance,
                          pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>& reg);

#endif
