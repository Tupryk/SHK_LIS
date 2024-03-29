import h5py
import trimesh
import numpy as np

def write_ply_with_decomposition_to_h5(ply_filename, h5_filename):
    """Reads a PLY file, stores centered mesh, performs convex decomposition, calculates inertia and yields .g File line for implementing in configuration.

    Args:
        ply_filename: Path to the PLY file.
        h5_filename: Path to the output H5 file.
    """
    mass = .1
    mesh =   trimesh.load(ply_filename, force='mesh')

    print("center of mass now" , mesh.center_mass)
    print("inertia now\n" , mesh.moment_inertia)

    transform = np.eye(4)

    U, D, V = np.linalg.svd(mesh.moment_inertia)

    # Ensure proper rotation  
    if np.linalg.det(V) < 0:
        V[:, -1] *= -1


    transform[0:3, 3] = V@(-mesh.center_mass)  # Move center of mass to origin

    transform[0:3, 0:3] = V  # Rotate the mesh to align with principal axes

    mesh.apply_transform(transform)

    print("\n\ncenter of mass after transformation" , mesh.center_mass)
    print("inertia after transformation\n" , mesh.moment_inertia, "\n")

    scaled_inertia = 1000*np.diag(mesh.moment_inertia)


    with h5py.File(h5_filename, 'w') as h:
        # Create groups for original mesh and decomposed meshes
        mesh_group = h.create_group("mesh")
        decomp_group = h.create_group("decomp")
        inertia_group = h.create_group("inertia")

        

        try:
            # Write original mesh data with colors (if available)
            mesh_group.create_dataset("vertices", data=mesh.vertices)
            mesh_group.create_dataset("faces", data=mesh.faces)
            mesh_colors = mesh.visual.vertex_colors # original mesh colors
            #mesh_group.create_dataset("colors", data=mesh_colors)

            inertia_group.create_dataset("mass", data=1)   # fix .1kg vorläufig
            inertia_group.create_dataset("inertia", data=scaled_inertia)  
            inertia_group.create_dataset("com", data=mesh.center_mass)  


            # Perform convex decomposition
            convex_hulls = mesh.convex_decomposition()

            colors = []
            parts = [0]
            vertices = np.asarray(convex_hulls[0].vertices)
            faces = np.asarray(convex_hulls[0].faces)
            vert_len = len(convex_hulls[0].vertices) 
            for i, part in enumerate(convex_hulls[1:]):                

                vertices = np.concatenate([vertices, part.vertices]) 
                faces = np.concatenate([faces, part.faces + vert_len])  
                parts.append(len(part.vertices)+parts[i])

                vert_len+=len(part.vertices)

                
            decomp_group.create_dataset("vertices", data=vertices)
            decomp_group.create_dataset("colors", data=np.array([0,0,255]))     #convex decomp fixed blue provisionally
            decomp_group.create_dataset("faces", data=faces)
            decomp_group.create_dataset("parts",  data=parts)


        except Exception as e:
            print(f"Error processing {ply_filename}: {e}")

    print(f"Original mesh and convex decomposed meshes using original colors saved to H5: {h5_filename}")
   

    frame_name = h5_file.split(".")[0]
    print("\n--- .g file line: ---")
    print(f"{frame_name}:  {{X:\"t(.4 .2 1.)\", mesh: <{h5_file}>, mass:{mass}, inertia:[{scaled_inertia[0]}, {scaled_inertia[1]}, {scaled_inertia[2]}]}}")


# Example usage
ply_file = "data/giraffe.ply"
h5_file = "giraffe.h5"
write_ply_with_decomposition_to_h5(ply_file, h5_file)