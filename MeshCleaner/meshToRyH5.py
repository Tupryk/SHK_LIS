from mesh_helper import *
import argparse


VISUALIZE = False
VERBOSE = 0
DECOMPOSE = False

def main():

    parser = argparse.ArgumentParser(description="This program demonstrates argument parsing with help option.")
    parser.add_argument("meshName", type=str, help="Filename of mesh you want to transform")
    parser.add_argument("-v", "--view", action="store_true", help="Launches into ry-view mode after mesh processing (optional).")
    parser.add_argument("-d", "--decomposecvx", action="store_true", help="Calculates convex decomposition with mesh and stores it in .g file (optional).")

    #parser.add_argument("--verbose", action="store_true", help="TODO add verbosity settings")

    args = parser.parse_args()

    file = args.meshName
    
    M = MeshHelper(file)

    M.autoScale()
    M.repair()

    if VERBOSE == 2: 
        print('  watertight:', M.mesh.is_watertight)
        print('  oriented:', M.mesh.is_winding_consistent)


    M.transformInertia()
    M.export_ply()
    M.export_mesh(False)
    M.createPoints()
    if args.decomposecvx:
        M.createDecomposition()
    M.export_h5(args.decomposecvx)
    gFile = M.export_scene(args.decomposecvx)
        
    print('=== done: ', file)
    
    if args.view:
        import robotic as ry
        import sys 
        try:
            C = ry.Config()
            C.watchFile(gFile)
        except KeyboardInterrupt:
            sys.exit(1)

if __name__ == "__main__":
    main()