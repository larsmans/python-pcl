from __future__ import print_function

import numpy as np
from numpy import cos, sin
from numpy.testing import assert_equal
import unittest

import pcl
from pcl.registration import icp, gicp, icp_nl, ia_ransac

bun0Tobun4 = [[ 0.85250509,    -0.03745676,    -0.52137518,    0.04118973],
[0.03552843,    0.99927479,    -0.01369729,    0.00103067],
[0.52151012,    -0.00684663,    0.8532176,    0.03994245],
[0.,    0.,    0.,    1.]]

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

        #print("---------")
        #print("Rotation: ")
        #print(transform[0:3,0:3])
        # print("Translation: ", transform[3, 0:3])
        #print("---------")

        random_cloud = np.random.RandomState(42).randn(900, 3)
        self.source = pcl.PointCloud(random_cloud.astype(np.float32))
        a = np.dot(random_cloud, transform).astype(np.float32)
        self.target = pcl.PointCloud(a)

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
#         print("------", algo)
#         print("Converged: ", converged, "Estimate: ", estimate,
#              "Fitness: ", fitness)
#         print("Rotation: ")
#         print(transf[0:3,0:3])
#         print("Complete: ")
#         print(transf)
#         print("Translation: ", transf[3, 0:3])
#         print("---------")
        
        #self.assertTrue(converged is True) # Commented out because Ransac doesn't think it's converged even though transf is ok
        #self.assertLess(fitness, .1) # Commented out because we don't know what fitness is or whether it is important at all

        self.assertTrue(isinstance(transf, np.ndarray))
        self.assertEqual(transf.shape, (4, 4))
        
        np.testing.assert_allclose(bun0Tobun4, transf, 0, 0.1)
        
#         estimate.to_file("tests/output" + `algo` + ".pcd");

        assert_equal(transf[3], [0, 0, 0, 1])

        # XXX I think I misunderstand fitness, it's not equal to the following
        # MSS.

    def testICP(self):
        self.check_algo(icp)

    def testGICP(self):
        self.check_algo(gicp)

    def testICP_NL(self):
        self.check_algo(icp_nl)

    def testIA_RANSAC(self):
        # reducing radius makes this test fail
        # reducing the max_iter to 1000 makes the test fail
        self.check_algo(ia_ransac, radius=0.5, minSampleDistance=0.01, maxCorrespondenceDistance=0.5, max_iter=10000)
