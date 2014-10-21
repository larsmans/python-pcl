#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh.h>

#include "registration_helper.h"

void mpcl_compute_fpfh(pcl::PointCloud<pcl::PointXYZRGB> &cloud_source_ptr, pcl::PointCloud<pcl::PointXYZRGB> &cloud_target_ptr,
    double searchRadius, pcl::PointCloud<pcl::FPFHSignature33> &feature_source_out, pcl::PointCloud<pcl::FPFHSignature33> &feature_target_out)
{
    // Initialize estimators for surface normals and FPFH features
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (searchRadius); // 0.05
    pcl::PointCloud<pcl::Normal> normals;

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setSearchMethod (tree);
    fpfh_est.setRadiusSearch (searchRadius); // 0.05

    // Estimate the FPFH features for the source cloud
    norm_est.setInputCloud (cloud_source_ptr.makeShared());
    norm_est.compute (normals);

    fpfh_est.setInputCloud (cloud_source_ptr.makeShared());
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (feature_source_out);

    // Estimate the FPFH features for the target cloud
    norm_est.setInputCloud (cloud_target_ptr.makeShared());
    norm_est.compute (normals);
    fpfh_est.setInputCloud (cloud_target_ptr.makeShared());
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (feature_target_out);
}

void mpcl_sample_consensus_initial_alignment_init(pcl::PointCloud<pcl::PointXYZRGB> &cloud_source_ptr,
                                            pcl::PointCloud<pcl::PointXYZRGB> &cloud_target_ptr,
                                            double searchRadius, double minSampleDistance, double maxCorrespondenceDistance,
                                            pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> &reg)
{
    pcl::PointCloud<pcl::FPFHSignature33> features_source, features_target;
    // estimate features
    mpcl_compute_fpfh(cloud_source_ptr, cloud_target_ptr, searchRadius, features_source, features_target);
    
    // Initialize Sample Consensus Initial Alignment (SAC-IA)
    reg.setMinSampleDistance (minSampleDistance); // 0.05
    reg.setMaxCorrespondenceDistance (maxCorrespondenceDistance); // 0.2
    reg.setSourceFeatures (features_source.makeShared ());
    reg.setTargetFeatures (features_target.makeShared ());
}
