"""
Run the registration algorithm on PCD files.
"""

import argparse
import pcl
import pcl.registration

def parse_args():
    parser = argparse.ArgumentParser(description='Registration for two PLY point clouds')
    parser.add_argument('-f','--function', help='Registration algorithm to run. Choose between gicp, icp, icp_nl, and ia_ransac. Defaults to gicp')
    parser.add_argument('source', metavar="SRC_PLY", help="Source PLY file")
    parser.add_argument('target', metavar="TARGET_PLY", help="Target PLY file to map source to")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    
    source = pcl.load(args.source)
    target = pcl.load(args.target)
    
    funcs = {
        'icp': pcl.registration.icp,
        'gicp': pcl.registration.gicp,
        'icp_nl': pcl.registration.icp_nl,
        'ia_ransac': pcl.registration.ia_ransac}
    
    if args.function in funcs:
        algo = funcs[args.function]
    else:
        algo = pcl.registration.gicp
        
    converged, transf, estimate, fitness = algo(source, target)
    
    print "------", algo.__name__, "-----"
    print "Converged: ", converged, "Fitness: ", fitness
    print "Rotation: "
    print transf[0:3,0:3]
    print "Translation: ", transf[3, 0:3]
    print "---------------"
