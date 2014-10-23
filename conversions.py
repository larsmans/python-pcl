'''
Created on Oct 22, 2014

@author: carlosm
'''

import liblas
import pcl
import numpy as np


def loadLas(lasFile):
    try:
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
    finally:
        las.close()

def writeLas(lasFile, pc):
    try:
        f = liblas.schema.Schema()
        f.time = False
        f.color = True

        h = liblas.header.Header()
        h.schema = f
        h.dataformat_id = 3
        h.minor_version = 2

        h.min = pc.to_array().min(axis=0)
        h.max = pc.to_array().max(axis=0)
        h.scale = [1.0, 1.0, 1.0]
        h.offset = [0., 0., 0.]

        las = liblas.file.File(lasFile, mode="w", header=h)

        for i in range(pc.size):
            pt = liblas.point.Point()
            pt.x,pt.y,pt.z, r,g,b = pc[i]
            pt.color = liblas.color.Color( red = int(r * 256), green = int(g * 256), blue = int(b * 256) )
            las.write(pt)

    finally:
        las.close()


def las2ply(lasFile, plyFile):
    pc, offset = loadLas(lasFile)
    pcl.save(pc, plyFile, format='PLY')

def ply2las(plyFile, lasFile):
    pc = pcl.load(plyFile, loadRGB=True)
    writeLas( lasFile, pc )

if __name__ == '__main__':
    plyFile = 'tests/10.ply'
    lasFile = 'tests/10.las.out'

    print 'From ply to las...'
    pc = pcl.load(plyFile, format='PLY', loadRGB=True)

    # print 'From las to ply...'
    # las2ply(lasFile, plyFile)



