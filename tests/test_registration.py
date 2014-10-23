from __future__ import print_function

import numpy as np
from numpy import cos, sin
import unittest

import pcl
from pcl.registration import icp, gicp, icp_nl, ia_ransac


class TestICP(unittest.TestCase):
    def setUpRandom(self):
        # Check if ICP can find a mild rotation.
        theta = [-.031, .4, .59]
        # theta = [-.031, .01, .05]
        rot_x = [[ 1,              0,             0             ],
                 [ 0,              cos(theta[0]), -sin(theta[0])],
                 [ 0,              sin(theta[0]),  cos(theta[0])]]
        rot_y = [[ cos(theta[1]),  0,              sin(theta[1])],
                 [ 0,              1,              0            ],
                 [-sin(theta[1]),  0,              cos(theta[1])]]
        rot_z = [[ cos(theta[2]), -sin(theta[1]),  0            ],
                 [ sin(theta[2]),  cos(theta[1]),  0            ],
                 [ 0,              0,              1            ]]
        transform = np.dot(rot_x, np.dot(rot_y, rot_z))

        print("---------")
        print("Rotation: ")
        print(transform[0:3,0:3])
        # print("Translation: ", transform[3, 0:3])
        print("---------")

        random_cloud = np.random.RandomState(42).randn(900, 3)
        # print(cloud.)
        self.source = pcl.PointCloud(random_cloud.astype(np.float32))
        self.target = pcl.PointCloud(np.dot(random_cloud, transform).astype(np.float32))
    
    def setUpBunny(self):
        self.source = pcl.PointCloud()
        self.source.from_file("tests/bun0.pcd")
        self.target = pcl.PointCloud()
        self.target.from_file("tests/bun4.pcd")

    def setUp(self):
        self.setUpBunny()
        # self.setUpRandom()

    def check_algo(self, algo, max_iter=1000, **kwargs):
        converged, transf, estimate, fitness = algo(self.source, self.target,
                                                    max_iter=max_iter, **kwargs)
        self.assertTrue(converged is True)
        self.assertLess(fitness, .1)

        self.assertTrue(isinstance(transf, np.ndarray))
        self.assertEqual(transf.shape, (4, 4))

        # XXX I think I misunderstand fitness, it's not equal to the following
        # MSS.
        mss = (np.linalg.norm(estimate.to_array()
                              - self.source.to_array(), axis=1) ** 2).mean()
        self.assertLess(mss, 1)

        # TODO check the actual transformation matrix.
        # print("------", algo)
        # print("Converged: ", converged, "Estimate: ", estimate,
        #      "Fitness: ", fitness)
        # print("Rotation: ")
        # print(transf[0:3,0:3])
        # print("Translation: ", transf[3, 0:3])
        # print("---------")

    def testICP(self):
        self.check_algo(icp)
   
    def testGICP(self):
        self.check_algo(gicp)

    def testICP_NL(self):
        self.check_algo(icp_nl)
        
    def testIA_RANSAC(self):
        self.check_algo(ia_ransac, radius=0.2, minSampleDistance=0.01, maxCorrespondenceDistance=0.5)
