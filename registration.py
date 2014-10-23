"""
Run the registration algorithm on PCD or ply files files.
"""

import argparse
import pcl
import pcl.registration
import liblas
import numpy as np
import time

def loadLas(lasFile):
    las = liblas.file.File(lasFile)
    nPoints = las.header.get_count()
    data_xyz = np.zeros((nPoints, 6), dtype=np.float64)
    min_point = np.array(las.header.get_min())
    max_point = np.array(las.header.get_max())
    offset = min_point + (max_point - min_point)/2

    for i,point in enumerate(las):
        data_xyz[i,0:3] = point.x,point.y,point.z
        data_xyz[i,0:3] -= offset
        data_xyz[i,3:6] = point.color.red,point.color.green,point.color.blue
        data_xyz[i,3:6] /= 256

    pc = pcl.PointCloudXYZRGB(data_xyz.astype(np.float32))
    return pc, offset

def parse_args():
    """ Parse arguments from the command-line using argparse """
    parser = argparse.ArgumentParser(description='Registration for two PLY point clouds')
    parser.add_argument('-f','--function', help='Registration algorithm to run. Choose between gicp, icp, icp_nl, and ia_ransac. Defaults to gicp')
    parser.add_argument('source', metavar="SOURCE", help="Source LAS file")
    parser.add_argument('target', metavar="TARGET", help="Target LAS file to map source to")
    return parser.parse_args()

def process_args(args):
    """ Read source, target and algorithm """
    print time.strftime("%H:%M:%S"), ": reading source", args.source
    # source = pcl.load(args.source)
    source, _ = loadLas(args.source)
    print time.strftime("%H:%M:%S"), ": reading target ", args.target
    target, _ = loadLas(args.target)
    # target = pcl.load(args.target)
    
    funcs = {
        'icp': pcl.registration.icp,
        'gicp': pcl.registration.gicp,
        'icp_nl': pcl.registration.icp_nl,
        'ia_ransac': pcl.registration.ia_ransac
    }
    
    if args.function in funcs:
        algo = funcs[args.function]
    else:
        algo = pcl.registration.gicp
    
    return source, target, algo

def print_output(algo, converged, transf, fitness):
    """ Print some output based on the algorithm output """
    
    print "Converged: ", converged, "Fitness: ", fitness
    print "Rotation: "
    print transf[0:3,0:3]
    print "Translation: ", transf[3, 0:3]
    print time.strftime("%H:%M:%S"), ": ---------------"

if __name__ == '__main__':
    args = parse_args()
    source, target, algo = process_args(args)
    
    # preprocess source and target
    print "Filtering source point cloud to leaf size 0.01"
    source_filter = source.make_voxel_grid_filter()
    source_filter.set_leaf_size(0.01, 0.01, 0.01)
    source = source_filter.filter()

    print "Filtering target point cloud to leaf size 0.01"
    target_filter = target.make_voxel_grid_filter()
    target_filter.set_leaf_size(0.01, 0.01, 0.01)
    target = target_filter.filter()
    
    print time.strftime("%H:%M:%S"), ": ------", algo.__name__, "-----"
    converged, transf, estimate, fitness = algo(source, target)
    print_output(algo, converged, transf, fitness)
