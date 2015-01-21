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

cdef extern from "minipcl.h":
    void mpcl_estimate_boundaries(cpp.PointCloud_t &,
                                  float, double, int, double,
                                  char *, size_t) except +


def estimate_boundaries(_pcl.BasePointCloud pc,
                        float angle_threshold,
                        double search_radius=-1,
                        int normal_ksearch=-1,
                        double normal_search_radius=-1):
    """Boundary estimation.

    Returns an array of booleans (true = boundary point); one per point in
    the point cloud pc.
    """
    cdef np.ndarray[np.uint8_t, ndim=1, mode='c'] out
    out = np.empty(len(pc), dtype=np.uint8)
    mpcl_estimate_boundaries(deref(pc.thisptr()), angle_threshold,
                             search_radius, normal_ksearch,
                             normal_search_radius, out.data, out.shape[0])
    return out.astype(np.bool, copy=False)
