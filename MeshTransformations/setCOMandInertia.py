import os
import trimesh
import numpy as np

files = []
files.append("data/notstopp.ply")

for file in files:

    print('file: ', file)

    ### load
    mesh =   trimesh.load(file, force='mesh')

    if mesh==None:
        continue

    print("center of mass now" , mesh.center_mass)
    print("inertia now\n" , mesh.moment_inertia)

    transform = np.eye(4)

    U, D, V = np.linalg.svd(mesh.moment_inertia)

    # Ensure proper rotation  
    if np.linalg.det(V) < 0:
        V[:, -1] *= -1


    transform[0:3, 3] = V@(-mesh.center_mass)  # Move center of mass toorigin

    transform[0:3, 0:3] = V  # Rotate the mesh to align with principal axes

    mesh.apply_transform(transform)

    print("center of mass after transformation" , mesh.center_mass)
    print("inertia after transformation\n" , mesh.moment_inertia)

    filebase = os.path.splitext(file)[0]

    print("scaled inertia \n" , 1000*np.diag(mesh.moment_inertia))

    ### export ply
    print('exporting as ply and mesh')
    mesh.export(filebase+'-.ply')

