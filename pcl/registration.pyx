#cython: embedsignature=True
#
# Copyright 2014 Netherlands eScience Center

from libcpp cimport bool

cimport numpy as np
import numpy as np

cimport _pcl
cimport pcl_defs as cpp

from cython.operator import dereference as deref
from shared_ptr cimport shared_ptr

np.import_array()

cdef extern from "pcl/point_types.h" namespace "pcl":
    cdef struct FPFHSignature33:
        FPFHSignature33()
        float fpfh[33]

cdef extern from "pcl/registration/registration.h" namespace "pcl" nogil:
    cdef cppclass Registration[Source, Target]:
        cppclass Matrix4:
            float *data()
        void align(cpp.PointCloud[Source] &) except +
        Matrix4 getFinalTransformation() except +
        double getFitnessScore() except +
        bool hasConverged() except +
        void setInputSource(cpp.PointCloudPtr_t) except +
        void setInputTarget(cpp.PointCloudPtr_t) except +
        void setMaximumIterations(int) except +

cdef extern from "pcl/registration/icp.h" namespace "pcl" nogil:
    cdef cppclass IterativeClosestPoint[Source, Target](Registration[Source, Target]):
        IterativeClosestPoint() except +

cdef extern from "pcl/registration/gicp.h" namespace "pcl" nogil:
    cdef cppclass GeneralizedIterativeClosestPoint[Source, Target](Registration[Source, Target]):
        GeneralizedIterativeClosestPoint() except +

cdef extern from "pcl/registration/icp_nl.h" namespace "pcl" nogil:
    cdef cppclass IterativeClosestPointNonLinear[Source, Target](Registration[Source, Target]):
        IterativeClosestPointNonLinear() except +

cdef extern from "pcl/registration/ia_ransac.h" namespace "pcl" nogil:
    cdef cppclass SampleConsensusInitialAlignment[Source, Target, Feature](Registration[Source, Target]):
        SampleConsensusInitialAlignment() except +

ctypedef SampleConsensusInitialAlignment[cpp.PointXYZRGB,cpp.PointXYZRGB,FPFHSignature33] SampleConsensusInitialAlignment_t
ctypedef shared_ptr[SampleConsensusInitialAlignment[cpp.PointXYZRGB,cpp.PointXYZRGB,FPFHSignature33]] SampleConsensusInitialAlignmentPtr_t

cdef extern from "registration_helper.h":
    void mpcl_sample_consensus_initial_alignment_init(
                            cpp.PointCloud_t, cpp.PointCloud_t,
                            double searchRadius, double minSampleDistance, double maxCorrespondenceDistance,
                            SampleConsensusInitialAlignment_t) except +

cdef object run(Registration[cpp.PointXYZRGB, cpp.PointXYZRGB] &reg,
                _pcl.PointCloud source, _pcl.PointCloud target, max_iter=None):
    reg.setInputSource(source.thisptr_shared)
    reg.setInputTarget(target.thisptr_shared)

    if max_iter is not None:
        reg.setMaximumIterations(max_iter)

    cdef _pcl.PointCloud result = _pcl.PointCloud()

    with nogil:
        reg.align(result.thisptr()[0])

    # Get transformation matrix and convert from Eigen to NumPy format.
    cdef Registration[cpp.PointXYZRGB, cpp.PointXYZRGB].Matrix4 mat
    mat = reg.getFinalTransformation()
    cdef np.ndarray[dtype=np.float32_t, ndim=2, mode='c'] transf
    cdef np.float32_t *transf_data

    transf = np.empty((4, 4), dtype=np.float32, order='c')
    transf_data = <np.float32_t *>np.PyArray_DATA(transf)

    for i in range(16):
        transf_data[i] = mat.data()[i]

    return reg.hasConverged(), transf, result, reg.getFitnessScore()

def icp(_pcl.PointCloud source, _pcl.PointCloud target, max_iter=None):
    """Align source to target using iterative closest point (ICP).

    Parameters
    ----------
    source : PointCloud
        Source point cloud.
    target : PointCloud
        Target point cloud.
    max_iter : integer, optional
        Maximum number of iterations. If not given, uses the default number
        hardwired into PCL.

    Returns
    -------
    converged : bool
        Whether the ICP algorithm converged in at most max_iter steps.
    transf : np.ndarray, shape = [4, 4]
        Transformation matrix.
    estimate : PointCloud
        Transformed version of source.
    fitness : float
        Sum of squares error in the estimated transformation.
    """
    cdef IterativeClosestPoint[cpp.PointXYZRGB, cpp.PointXYZRGB] icp
    return run(icp, source, target, max_iter)


def gicp(_pcl.PointCloud source, _pcl.PointCloud target, max_iter=None):
    """Align source to target using generalized iterative closest point (GICP).

    Parameters
    ----------
    source : PointCloud
        Source point cloud.
    target : PointCloud
        Target point cloud.
    max_iter : integer, optional
        Maximum number of iterations. If not given, uses the default number
        hardwired into PCL.

    Returns
    -------
    converged : bool
        Whether the ICP algorithm converged in at most max_iter steps.
    transf : np.ndarray, shape = [4, 4]
        Transformation matrix.
    estimate : PointCloud
        Transformed version of source.
    fitness : float
        Sum of squares error in the estimated transformation.
    """
    cdef GeneralizedIterativeClosestPoint[cpp.PointXYZRGB, cpp.PointXYZRGB] gicp
    return run(gicp, source, target, max_iter)


def icp_nl(_pcl.PointCloud source, _pcl.PointCloud target, max_iter=None):
    """Align source to target using generalized non-linear ICP (ICP-NL).

    Parameters
    ----------
    source : PointCloud
        Source point cloud.
    target : PointCloud
        Target point cloud.

    max_iter : integer, optional
        Maximum number of iterations. If not given, uses the default number
        hardwired into PCL.

    Returns
    -------
    converged : bool
        Whether the ICP algorithm converged in at most max_iter steps.
    transf : np.ndarray, shape = [4, 4]
        Transformation matrix.
    estimate : PointCloud
        Transformed version of source.
    fitness : float
        Sum of squares error in the estimated transformation.
    """
    cdef IterativeClosestPointNonLinear[cpp.PointXYZRGB, cpp.PointXYZRGB] icp_nl
    return run(icp_nl, source, target, max_iter)
	
def ia_ransac(_pcl.PointCloud source, _pcl.PointCloud target, radius=0.05, minSampleDistance=0.05, maxCorrespondenceDistance=0.2, max_iter=None):
    """
    An implementation of the initial alignment algorithm described in section IV
    of "Fast Point Feature Histograms (FPFH) for 3D Registration," Rusu et al. 

    Parameters
    ----------
    source : PointCloud
        Source point cloud.
    target : PointCloud
        Target point cloud.

    max_iter : integer, optional
        Maximum number of iterations. If not given, uses the default number
        hardwired into PCL.

    Returns
    -------
    converged : bool
        Whether the ICP algorithm converged in at most max_iter steps.
    estimate : PointCloud
        Transformed version of source.
    fitness : float
        Sum of squares error in the estimated transformation.
    """
    cdef SampleConsensusInitialAlignment[cpp.PointXYZRGB, cpp.PointXYZRGB, FPFHSignature33] ia_ransac
    mpcl_sample_consensus_initial_alignment_init(deref(source.thisptr()), deref(target.thisptr()), radius, minSampleDistance, maxCorrespondenceDistance, ia_ransac)
    return run(ia_ransac, source, target, max_iter)
