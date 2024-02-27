import trimesh
import numpy as np

file = "data/giraffe.ply"


print('file: ', file)


mesh =   trimesh.load(file, force='mesh')
meshes = mesh.convex_decomposition()


print("---")
concat_mesh = trimesh.util.concatenate(meshes)
concat_mesh.export('convex.ply')

