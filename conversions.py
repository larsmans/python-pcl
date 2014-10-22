'''
Created on Oct 22, 2014

@author: carlosm
'''

import liblas
import pcl
import numpy as np

def las2ply(lasFile, plyFile):
    las = liblas.file.File(lasFile)
    nPoints = las.header.get_count()
    data_xyz = np.zeros((nPoints, 3))
    scale = las.header.scale

    for i,point in enumerate(las):
        data_xyz[i,:] = point.x,point.y,point.z
        data_xyz[i,:] /= scale 

    pc = pcl.PointCloud(data_xyz.astype(np.float32))
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
#    lasFile = 'tests/Rome-000062.las'
#    plyFile = 'tests/Rome-000062.ply'
#    las2ply(lasFile, plyFile)

    plyFile = 'tests/rock.ply'
    lasFile = 'tests/rock.las'

    print 'converting forward...'
    ply2las(plyFile,lasFile)

    print 'converting back...'
    plyFile = 'tests/rock2.ply'
    las2ply(lasFile, plyFile)



