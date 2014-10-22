# XXX do a more specific import!
from ._pcl import *

import sys


class BasePyPointCloud(BasePointCloud):
    def __repr__(self):
        return "<%s of %d points>" % (type(self).__name__, self.size)

    # Pickle support. XXX this copies the entire pointcloud; it would be nice
    # to have an asarray member that returns a view, or even better, implement
    # the buffer protocol (https://docs.python.org/c-api/buffer.html).
    def __reduce__(self):
        return type(self), (self.to_array(),)

    def to_list(self):
        """Return this object as a list of tuples."""
        return self.to_array().tolist()

class PointCloud(BasePyPointCloud):
    def __getitem__(self, idx):
        p_x, p_y, p_z, p_r, p_g, p_b = super(BasePyPointCloud, self).__getitem__(idx)
        return p_x, p_y, p_z
    
    """3-d point cloud (no color information)."""
    def get_point(self, row, col):
        """Return point (3-tuple) at the given row/column."""
        return self._get_point(row, col)[:3]

    def to_array(self):
        """Return this object as a 2D numpy array (float32)."""
        return self._to_array(np.empty((self.size, 3), dtype=np.float32))

class PointCloudXYZRGB(BasePyPointCloud):
    """3-d point cloud (no color information)."""
    def get_point(self, row, col):
        """Return point (3-tuple) at the given row/column."""
        return self._get_point(row, col)
    
    """3-d point with color information."""

    def to_array(self):
        """Return this object as a 2D numpy array (float32)."""
        return self._to_array(np.empty((self.size, 6), dtype=np.float32))

def load(path, format=None):
    """Load pointcloud from path.

    Currently supports PCD and PLY files.

    Format should be "pcd", "ply", or None to infer from the pathname.
    """
    format = _infer_format(path, format)
    p = PointCloudXYZRGB()
    try:
        loader = getattr(p, "_from_%s_file" % format)
    except AttributeError:
        raise ValueError("unknown file format %s" % format)
    if loader(_encode(path)):
        raise IOError("error while loading pointcloud from %r (format=%r)"
                      % (path, format))
    return p

def save(cloud, path, format=None, binary=False):
    """Save pointcloud to file.

    Format should be "pcd", "ply", or None to infer from the pathname.
    """
    format = _infer_format(path, format)
    try:
        dumper = getattr(cloud, "_to_%s_file" % format)
    except AttributeError:
        raise ValueError("unknown file format %s" % format)
    if dumper(_encode(path), binary):
        raise IOError("error while saving pointcloud to %r (format=%r)"
                      % (path, format))

def _encode(path):
    # Encode path for use in C++.
    if isinstance(path, bytes):
        return path
    else:
        return path.encode(sys.getfilesystemencoding())


def _infer_format(path, format):
    if format is not None:
        return format.lower()

    for candidate in ["pcd", "ply"]:
        if path.endswith("." + candidate):
            return candidate

    raise ValueError("Could not determine file format from pathname %s" % path)
