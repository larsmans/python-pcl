from libc.stddef cimport size_t

from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool

from shared_ptr cimport shared_ptr
from vector cimport vector as vector2

cdef extern from "Eigen/Eigen" namespace "Eigen" nogil:
    cdef cppclass Vector4f:
        float *data()
    cdef cppclass Matrix4f:
        float *data()
    cdef cppclass Quaternionf:
        float w()
        float x()
        float y()
        float z()
    cdef cppclass aligned_allocator[T]:
        pass

cdef extern from "pcl/point_cloud.h" namespace "pcl":
    cdef cppclass PointCloud[T]:
        PointCloud() except +
        PointCloud(unsigned int, unsigned int) except +
        unsigned int width
        unsigned int height
        bool is_dense
        void resize(size_t) except +
        size_t size()
        #T& operator[](size_t)
        #T& at(size_t) except +
        #T& at(int, int) except +
        shared_ptr[PointCloud[T]] makeShared()

        Quaternionf sensor_orientation_
        Vector4f sensor_origin_

cdef extern from "indexing.hpp":
    # Use these instead of operator[] or at.
    PointXYZRGBNormal *getptr(PointCloud[PointXYZRGBNormal] *, size_t)
    PointXYZRGBNormal *getptr_at(PointCloud[PointXYZRGBNormal] *, size_t) except +
    PointXYZRGBNormal *getptr_at(PointCloud[PointXYZRGBNormal] *, int, int) except +

cdef extern from "pcl/point_types.h" namespace "pcl":
    cdef struct PointXYZRGBNormal:
        PointXYZRGBNormal()
        float x
        float y
        float z
        float rgb
    cdef struct Normal:
        pass

cdef extern from "pcl/features/normal_3d.h" namespace "pcl":
    cdef cppclass NormalEstimation[T, N]:
        NormalEstimation()

cdef extern from "pcl/segmentation/sac_segmentation.h" namespace "pcl":
    cdef cppclass SACSegmentationFromNormals[T, N]:
        SACSegmentationFromNormals()
        void setOptimizeCoefficients (bool)
        void setModelType (SacModel)
        void setMethodType (int)
        void setNormalDistanceWeight (float)
        void setMaxIterations (int)
        void setDistanceThreshold (float)
        void setRadiusLimits (float, float)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void setInputNormals (shared_ptr[PointCloud[N]])
        void setEpsAngle (double ea)
        void segment (PointIndices, ModelCoefficients)

    cdef cppclass SACSegmentation[T]:
        void setOptimizeCoefficients (bool)
        void setModelType (SacModel)
        void setMethodType (int)
        void setDistanceThreshold (float)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void segment (PointIndices, ModelCoefficients)

ctypedef SACSegmentation[PointXYZRGBNormal] SACSegmentation_t
ctypedef SACSegmentationFromNormals[PointXYZRGBNormal,Normal] SACSegmentationNormal_t

cdef extern from "pcl/surface/mls.h" namespace "pcl":
    cdef cppclass MovingLeastSquares[I,O]:
        MovingLeastSquares()
        void setInputCloud (shared_ptr[PointCloud[I]])
        void setSearchRadius (double)
        void setPolynomialOrder(bool)
        void setPolynomialFit(int)
        void process(PointCloud[O] &) except +

ctypedef MovingLeastSquares[PointXYZRGBNormal,PointXYZRGBNormal] MovingLeastSquares_t

cdef extern from "pcl/search/kdtree.h" namespace "pcl::search":
    cdef cppclass KdTree[T]:
        KdTree()

ctypedef aligned_allocator[PointXYZRGBNormal] aligned_allocator_t 
ctypedef vector2[PointXYZRGBNormal, aligned_allocator_t] AlignedPointTVector_t

cdef extern from "pcl/octree/octree_pointcloud.h" namespace "pcl::octree":
    cdef cppclass OctreePointCloud[T]:
        OctreePointCloud(double)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void defineBoundingBox()
        void defineBoundingBox(double, double, double, double, double, double)
        void addPointsFromInputCloud()
        void deleteTree()
        bool isVoxelOccupiedAtPoint(double, double, double)
        int getOccupiedVoxelCenters(AlignedPointTVector_t)	
        void deleteVoxelAtPoint(PointXYZRGBNormal)

ctypedef OctreePointCloud[PointXYZRGBNormal] OctreePointCloud_t

cdef extern from "pcl/octree/octree_search.h" namespace "pcl::octree":
    cdef cppclass OctreePointCloudSearch[T]:
        OctreePointCloudSearch(double)
        int radiusSearch (PointXYZRGBNormal, double, vector[int], vector[float], unsigned int)

ctypedef OctreePointCloudSearch[PointXYZRGBNormal] OctreePointCloudSearch_t

cdef extern from "pcl/ModelCoefficients.h" namespace "pcl":
    cdef struct ModelCoefficients:
        vector[float] values

cdef extern from "pcl/PointIndices.h" namespace "pcl":
    #FIXME: I made this a cppclass so that it can be allocated using new (cython barfs otherwise), and
    #hence passed to shared_ptr. This is needed because if one passes memory allocated
    #using malloc (which is required if this is a struct) to shared_ptr it aborts with
    #std::bad_alloc
    #
    #I don't know if this is actually a problem. It is cpp and there is no incompatibility in
    #promoting struct to class in this instance
    cdef cppclass PointIndices:
        vector[int] indices

ctypedef PointIndices PointIndices_t
ctypedef shared_ptr[PointIndices] PointIndicesPtr_t

cdef extern from "pcl/io/pcd_io.h" namespace "pcl::io":
    int load(string file_name, PointCloud[PointXYZRGBNormal] &cloud) nogil except +
    int loadPCDFile(string file_name,
                    PointCloud[PointXYZRGBNormal] &cloud) nogil except +
    int savePCDFile(string file_name, PointCloud[PointXYZRGBNormal] &cloud,
                    bool binary_mode) nogil except +

cdef extern from "pcl/io/ply_io.h" namespace "pcl::io":
    int loadPLYFile(string file_name,
                    PointCloud[PointXYZRGBNormal] &cloud) nogil except +
    int savePLYFile(string file_name, PointCloud[PointXYZRGBNormal] &cloud,
                    bool binary_mode) nogil except +

#http://dev.pointclouds.org/issues/624
#cdef extern from "pcl/io/ply_io.h" namespace "pcl::io":
#    int loadPLYFile (string file_name, PointCloud[PointXYZRGBNormal] cloud)
#    int savePLYFile (string file_name, PointCloud[PointXYZRGBNormal] cloud, bool binary_mode)

cdef extern from "pcl/sample_consensus/model_types.h" namespace "pcl":
    cdef enum SacModel:
        SACMODEL_PLANE
        SACMODEL_LINE
        SACMODEL_CIRCLE2D
        SACMODEL_CIRCLE3D
        SACMODEL_SPHERE
        SACMODEL_CYLINDER
        SACMODEL_CONE
        SACMODEL_TORUS
        SACMODEL_PARALLEL_LINE
        SACMODEL_PERPENDICULAR_PLANE
        SACMODEL_PARALLEL_LINES
        SACMODEL_NORMAL_PLANE
        #SACMODEL_NORMAL_SPHERE
        SACMODEL_REGISTRATION
        SACMODEL_PARALLEL_PLANE
        SACMODEL_NORMAL_PARALLEL_PLANE
        SACMODEL_STICK

cdef extern from "pcl/sample_consensus/method_types.h" namespace "pcl":
    cdef enum:
        SAC_RANSAC = 0
        SAC_LMEDS = 1
        SAC_MSAC = 2
        SAC_RRANSAC = 3
        SAC_RMSAC = 4
        SAC_MLESAC = 5
        SAC_PROSAC = 6

ctypedef PointCloud[PointXYZRGBNormal] PointCloud_t
ctypedef PointCloud[Normal] PointNormalCloud_t
ctypedef shared_ptr[PointCloud[PointXYZRGBNormal]] PointCloudPtr_t

cdef extern from "pcl/common/transforms.h" namespace "pcl" nogil:
    cdef void transformPointCloud(PointCloud_t &, PointCloud_t &, Matrix4f)

cdef extern from "pcl/filters/statistical_outlier_removal.h" namespace "pcl":
    cdef cppclass StatisticalOutlierRemoval[T]:
        StatisticalOutlierRemoval()
        int getMeanK()
        void setMeanK (int nr_k)
        double getStddevMulThresh()
        void setStddevMulThresh (double std_mul)
        bool getNegative()
        void setNegative (bool negative)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void filter(PointCloud[T] &c)

ctypedef StatisticalOutlierRemoval[PointXYZRGBNormal] StatisticalOutlierRemoval_t

cdef extern from "pcl/filters/voxel_grid.h" namespace "pcl":
    cdef cppclass VoxelGrid[T]:
        VoxelGrid()
        void setLeafSize (float, float, float)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void filter(PointCloud[T] c)

ctypedef VoxelGrid[PointXYZRGBNormal] VoxelGrid_t

cdef extern from "pcl/filters/passthrough.h" namespace "pcl":
    cdef cppclass PassThrough[T]:
        PassThrough()
        void setFilterFieldName (string field_name)
        void setFilterLimits (float, float)
        void setInputCloud (shared_ptr[PointCloud[T]])
        void filter(PointCloud[T] c)

ctypedef PassThrough[PointXYZRGBNormal] PassThrough_t

cdef extern from "pcl/kdtree/kdtree_flann.h" namespace "pcl":
    cdef cppclass KdTreeFLANN[T]:
        KdTreeFLANN()
        void setInputCloud (shared_ptr[PointCloud[T]])
        int nearestKSearch (PointCloud[T],
          int, int, vector[int], vector[float])

ctypedef KdTreeFLANN[PointXYZRGBNormal] KdTreeFLANN_t
