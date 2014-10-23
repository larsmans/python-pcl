# Introduction to python-pcl

We have been implementing python binding to the `pointcloud <http://pointclouds.org/>`_ library (PCL). This is a work in progress and currently the following modules/functionalities of the API are wrapped: 

 * I/O and integration; saving and loading PCD (Point Cloud Data) files.
 * segmentation
 * SAC (Simple Alignment Consensus)
 * smoothing
 * filtering
 * registration 

We aim to wrap the following modules/functionalities:

 * feature extraction 
 * ....
  
Note that all methods operate in Cartesian coordinate system i.e. (x, y, z). Point cloud data comes in two formats (x,y,z) and (x,y,z, R, G, B). These methods work on both.

some useful links to PCL:


# *Disclaimer*

This release has been tested on: 

Ubuntu 13.10 with,

 * Python 2.7.5
 * pcl 1.7.1
 * Cython 0.20.2

CentOS 6.5 with,

 * Python 2.6.6
 * pcl 1.6.0
 * Cython 0.20.2

Fedora 20 with,
 
 * Python 2.7.5
 * pcl 1.7.0
 * Cython 0.21.1

The instruction below holds for Ubuntu and Fedora.
# preparatory steps to use python-pcl
## dependencies
You need to have the following packages installed on your system:

**In Ubuntu**

````
sudo apt-get install cmake libgdal-dev pcl
sudo apt-get install cmake libgdal-dev libgeotiff-dev
````
**In Fedora**

  
System-wide:

* ```` sudo yum install gdal-devel libgeotiff-devel ````
* ```` sudo yum install boost-devel ````
* ```` sudo yum install gcc-c++ ````

Python libraries:

* ```` sudo yum install python-pip  python-devel ````
* ```` sudo pip install numpy ````
* ```` sudo pip install scipy ````

LibLAS:

```` 
git clone git://github.com/libLAS/libLAS.git liblas
cd liblas/
mkdir makefiles
cd makefiles/
cmake -G "Unix Makefiles" ../ --GEOTIFF_INCLUDE_DIR=/usr/include/libgeotiff/
make
sudo make install
````
NOTE: You might need to export LD_LIBRARY_PATH i.e in your .bashrc file, add this line:

```` export LD_LIBRARY_PATH=/usr/local/lib ````

PCL:

```` sudo yum install pcl pcl-devel pcl-tools pcl-doc ````


## python-pcl

Clone repository, compile and test that it works:
````make
git clone 'http://github.com/larsmans/python-pcl'
cd python-pcl/
make
python -m unittest discover tests
````

Now you should be able to use python-pcl modules and functionalities as explained below.

# Quick Tutorial: how to use python-pcl 

## PointCloud

It instantiates a point could object. It also provides functionality for manipulating point cloud data in a numpy array. 

````
pcl.PointCloud()
pcl.PointCloudXYZRGB()
````

**example**
````
import numpy as np
import pcl
arr = np.zeros((100, 3)).astype(np.float32)
pc = pcl.PointCloud()
pc.from_array(arr)
pc.to_array()

````

````
import numpy as np
import pcl
arr = np.zeros((100, 6)).astype(np.float32)
pc = pcl.PointCloudXYZRGB()
pc.from_array(arr)
pc.to_array()

````

## I/O
It helps loading and saving point cloud files. 

````
pcl.load(path, format=None, loadRGB=False)
pcl.save(cloud, path, format=None, binary=False)
````

**example**

Load a point cloud from PCD file:

````
import pcl
pc = pcl.load('tests/bun0.pcd')
````

Load a point cloud from PLY file:

````
import pcl
pc = pcl.load('tests/rock2.ply')
````

Load a point cloud from PLY file

````
import pcl
pc = pcl.load('tests/rock2.ply', loadRGB=True)
````

Save point cloud to PLY file (save to PCD is also supported).

````
pcl.save(pc, 'tests/tmp.ply', format='PLY')
````

## segmentation


````
pcl.Segmentation()
...

````

**example**


````
import pcl
pc = pcl.load('some_cloud.pcd')
seg = pc.make_segmenter()
seg.set_optimize_coefficients (True)
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold (0.01)
indices, model = seg.segment()
pc2 = pcl.PointCloud()
pc2.from_array(pc.to_array()[indices,:])
pcl.save(pc2, 'other_cloud.ply', format='PLY')

````


````
import pcl
pc = pcl.load("some_cloud.pcd")
seg = pc.make_segmenter_normals(50)
seg.set_optimize_coefficients (True);
seg.set_model_type (pcl.SACMODEL_CYLINDER)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_normal_distance_weight (0.1)
seg.set_max_iterations (10000)
seg.set_distance_threshold (0.05)
seg.set_radius_limits (0, 0.1)
indices, model = seg.segment()
pc2 = pcl.PointCloud() # to get a second point cloud of the segmented point cloud
pc2.from_array(pc.to_array()[indices,:])
pcl.save(pc2, 'other_cloud.ply', format='PLY') 

````

## SAC 

**example**
## smoothing
smoothing does ... and it is implemented as *....*

**example**
````
    import pcl
    p = pcl.load("C/table_scene_lms400.pcd")
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (50)
    fil.set_std_dev_mul_thresh (1.0)
    fil.filter().to_file("inliers.pcd")
````    

## filtering

...

**example**


## registration

In registration module, we have wrapped four registration method ICP (Iterative Closest Point), generalized ICP, generalized non-linear ICP, initial alignment RANSAC ():

````
pcl.registration.icp(BasePointCloud source, BasePointCloud target, max_iter = None)
pcl.registartion.gicp(BasePointCloud source, BasePointCloud target, max_iter = None)
pcl.registartion.icp_nl(BasePointCloud source, BasePointCloud target, max_iter = None)
pcl.registartion.ia_ransac(BasePointCloud source, BasePointCloud target, max_iter = None, radius=0.05, minSampleDistance=0.05, maxCorrespondenceDistance=0.2)
````

**example**  

````
import pcl
import pcl.registration

sourceFile = 'tests/color.ply'
targetFile = 'tests/color.ply'

source = pcl.load(sourceFile)
target = pcl.load(targetFile)

converged, transf, estimate, fitness = pcl.registration.icp(source, target)
````

````
import pcl
import pcl.registration

sourceFile = 'tests/color.ply'
targetFile = 'tests/color.ply'

source = pcl.load(sourceFile)
target = pcl.load(targetFile)
converged, transf, estimate, fitness = pcl.registration.gicp(source, target)
````

````
import pcl
import pcl.registration

sourceFile = 'tests/color.ply'
targetFile = 'tests/color.ply'

source = pcl.load(sourceFile)
target = pcl.load(targetFile)

converged, transf, estimate, fitness = pcl.registration.icp_nl(source, target)
````

````
import pcl
import pcl.registration

sourceFile = 'tests/color.ply'
targetFile = 'tests/color.ply'

source = pcl.load(sourceFile)
target = pcl.load(targetFile)

converged, transf, estimate, fitness = pcl.registration.ia_ransac(source, target)
````


## more with numpy helper
numpy helper does ...

**example**

    import pcl
    import numpy as np
    p = pcl.PointCloud(np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32))
    seg = p.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    indices, model = seg.segment()


API Documentation
=================

.. autosummary::
   pcl.PointCloud
   pcl.Segmentation
   pcl.SegmentationNormal
   pcl.StatisticalOutlierRemovalFilter
   pcl.MovingLeastSquares
   pcl.PassThroughFilter
   pcl.VoxelGridFilter



