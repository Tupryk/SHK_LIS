import os
import trimesh
import numpy as np
from tqdm import tqdm
from PIL import Image
from mesh_helper import *

def obj2ply(obj_file: str, ply_out: str, scale: float=1.0, texture_path: str="none", toH5: bool = True) -> bool:

    tri_obj = trimesh.load(obj_file)
    if hasattr(tri_obj.visual, 'to_color'):

        if texture_path == "none":
            vertex_colors_visual = tri_obj.visual.to_color()
            tri_obj.visual = vertex_colors_visual
        elif texture_path:
            try:
                texture_image = Image.open(texture_path)
                
                texture = trimesh.visual.TextureVisuals(image=texture_image, uv=tri_obj.visual.uv)
                tri_obj.visual = texture
    
                
                texture = Image.open(texture_path)
                texture = np.array(texture).astype(float) / 255.0
                
                if texture.shape[-1] == 3:
                    alpha = np.ones((texture.shape[0], texture.shape[1], 1))
                    texture = np.concatenate([texture, alpha], axis=-1)
                
                if not hasattr(tri_obj.visual, 'uv'):
                    raise ValueError("Mesh does not have UV coordinates!")
                
                uv_coords = tri_obj.visual.uv
                
                uv_coords_copy = uv_coords.copy()
                uv_coords_copy[:, 1] = 1 - uv_coords_copy[:, 1]
                
                pixel_coords = np.zeros_like(uv_coords_copy)
                pixel_coords[:, 0] = uv_coords_copy[:, 0] * (texture.shape[1] - 1)
                pixel_coords[:, 1] = uv_coords_copy[:, 1] * (texture.shape[0] - 1)
                pixel_coords = pixel_coords.astype(int)
                
                vertex_colors = texture[pixel_coords[:, 1], pixel_coords[:, 0]]
                
                tri_obj.visual = trimesh.visual.ColorVisuals(
                    mesh=tri_obj,
                    vertex_colors=vertex_colors
                )
                
                
            except:
                pass
        if scale != 1.0:
            print(scale)
            scaling_mat = scale * np.eye(4)
            scaling_mat[-1, -1] = 1.0
            tri_obj.apply_transform(scaling_mat)

        tri_obj.export(ply_out)
        print(f"Converted {obj_file}")
        
        if toH5:
            M = MeshHelper(ply_out)
            transform_mat = M.transformInertia()
            M.createPoints()
            M.createDecomposition()
            M.export_h5(True)
            return True, M.mesh.center_mass.tolist(), np.diagonal(M.mesh.moment_inertia).tolist(), transform_mat
        
        return True

    else:
        print(f"Failed on {obj_file}")
        if toH5:
            return False, False, False, False
        return False
        
    

if __name__ == "__main__":

    root_path = "aigen_objs"

    for root, dirs, files in tqdm(os.walk(root_path)):
        for file in files:
            if file.endswith('.obj') and not "collision" in file:
                
                ply_dir = os.path.join(f"./rai_plys/{root}".replace("/visual", ""))
                if not os.path.exists(ply_dir):
                    os.makedirs(ply_dir)

                ply_path = os.path.join(ply_dir, file.replace(".obj", ".ply"))
                obj_path = os.path.join(root, file)

                try:
                    obj2ply(obj_path, ply_path, scale=.2)
                except:
                    print(f"Failed to convert {obj_path}")