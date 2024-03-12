import h5py
import trimesh
import numpy as np


def write_ply_with_decomposition_to_h5(ply_filename, h5_filename):
    """Reads a PLY file, stores original mesh, performs convex decomposition, and uses original mesh colors for decomposed parts.

    Args:
        ply_filename: Path to the PLY file.
        h5_filename: Path to the output H5 file.
    """

    with h5py.File(h5_filename, 'w') as h:
        # Create groups for original mesh and decomposed meshes
        mesh_group = h.create_group("mesh")
        decomp_group = h.create_group("decomp")


        try:
            # Load mesh using trimesh
            mesh = trimesh.load_mesh(ply_filename)

            # Write original mesh data with colors (if available)
            mesh_group.create_dataset("vertices", data=mesh.vertices)
            mesh_group.create_dataset("faces", data=mesh.faces)
            mesh_colors = mesh.visual.vertex_colors # Store original mesh colors
            mesh_group.create_dataset("colors", data=mesh_colors)


            # Perform convex decomposition
            convex_hulls = mesh.convex_decomposition()

            a = [0]
            colors = []


            vertices = np.asarray(convex_hulls[0].vertices)
            faces = np.asarray(convex_hulls[0].faces)
            vert_len = len(convex_hulls[0].vertices)  # Initialize vert_len with the length of vertices in the first part
            for i, part in enumerate(convex_hulls[1:]):                

                vertices = np.concatenate([vertices, part.vertices]) 
                faces = np.concatenate([faces, part.faces + vert_len])  
                a.append(len(part.vertices)+a[i])

                vert_len+=len(part.vertices)

                
            decomp_group.create_dataset("vertices", data=vertices)
            decomp_group.create_dataset("colors", data=np.array([0,0,255]))
            decomp_group.create_dataset("faces", data=faces)
            decomp_group.create_dataset("parts",  data=a)


        except Exception as e:
            print(f"Error processing {ply_filename}: {e}")

    print(f"Original mesh and convex decomposed meshes using original colors saved to H5: {h5_filename}")


# Example usage
ply_file = "data/giraffe--.ply"
h5_file = "coolioo.h5"
write_ply_with_decomposition_to_h5(ply_file, h5_file)
