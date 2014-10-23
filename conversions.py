'''
Created on Oct 22, 2014

@author: carlosm
'''

import liblas
import pcl
import numpy as np

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

def las2ply(lasFile, plyFile):
    pc, offset = loadLas(lasFile)
    pcl.save(pc, plyFile, format='PLY')

def ply2las(plyFile, lasFile):
    pc = pcl.load(plyFile)
    f = liblas.file.File(lasFile,mode='w')
    for i in range(pc.size):
        pt = liblas.point.Point()
        pt.x,pt.y,pt.z = pc[i]
        f.write(pt)
    f.close()

if __name__ == '__main__':
    plyFile = 'tests/10.ply'
    lasFile = 'tests/10.las'

    print 'From las to ply...'
    las2ply(lasFile, plyFile)



