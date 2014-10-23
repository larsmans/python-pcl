#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh.h>

#include "registration_helper.h"

using pcl::FPFHSignature33;
using pcl::PointCloud;
using pcl::PointXYZRGB;

#if 0
static void mpcl_print_fpfhfloat(const PointCloud<FPFHSignature33> &feature_source,
                                 const PointCloud<FPFHSignature33> &feature_target)
{
    printf("Features source size %zu\n", feature_source.size());
    printf("Features target size %zu\n", feature_target.size());
    printf("Features source point 0 - in func: ");

    union {
        FPFHSignature33 pclsig;
        float floatsig[33];
    } p0 = { .pclsig = feature_source[0] };
    for (int i = 0; i < 33; i++) {
        printf("%f ", p0.floatsig[i]);
    }
    printf("\n");    
}
#endif
    
void mpcl_compute_fpfh(PointCloud<PointXYZRGB> &cloud_source_ptr,
                       PointCloud<PointXYZRGB> &cloud_target_ptr,
                       double searchRadius,
                       PointCloud<FPFHSignature33> &feature_source_out,
                       PointCloud<FPFHSignature33> &feature_target_out)
{
    // Initialize estimators for surface normals and FPFH features
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());

    pcl::NormalEstimation<PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (searchRadius); // 0.05
    PointCloud<pcl::Normal> normals;

    pcl::FPFHEstimation<PointXYZRGB, pcl::Normal, FPFHSignature33> fpfh_est;
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

void mpcl_sac_ia_init(PointCloud<PointXYZRGB> &cloud_source_ptr,
                      PointCloud<PointXYZRGB> &cloud_target_ptr,
                      double searchRadius, double minSampleDistance,
                      double maxCorrespondenceDistance,
                      pcl::SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB,
                                                           FPFHSignature33> &reg)
{
    PointCloud<FPFHSignature33> features_source, features_target;
    // estimate features
    mpcl_compute_fpfh(cloud_source_ptr, cloud_target_ptr, searchRadius,
                      features_source, features_target);
    
    // Initialize Sample Consensus Initial Alignment (SAC-IA)
    reg.setMinSampleDistance (minSampleDistance); // 0.05
    reg.setMaxCorrespondenceDistance (maxCorrespondenceDistance); // 0.2
    reg.setSourceFeatures (features_source.makeShared ());
    reg.setTargetFeatures (features_target.makeShared ());
}
