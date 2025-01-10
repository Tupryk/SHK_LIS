import os
from tqdm import tqdm
import trimesh


def obj2ply(obj_file: str, ply_out: str) -> bool:

    tri_obj = trimesh.load(obj_file)
    if hasattr(tri_obj.visual, 'to_color'):

        vertex_colors_visual = tri_obj.visual.to_color()

        tri_obj.visual = vertex_colors_visual

        tri_obj.export(ply_out)

        print(f"Converted {obj_file}")
        return True

    else:
        print(f"Failed on {obj_file}")
        return False


if __name__ == "__main__":

    root_path = "fixtures/accessories"

    for root, dirs, files in tqdm(os.walk(root_path)):
        for file in files:
            if file.endswith('.obj') and not "collision" in file:
                
                ply_dir = os.path.join(f"./rai_plys/{root}".replace("/visual", ""))
                if not os.path.exists(ply_dir):
                    os.makedirs(ply_dir)

                ply_path = os.path.join(ply_dir, file.replace(".obj", ".ply"))
                obj_path = os.path.join(root, file)

                try:
                    obj2ply(obj_path, ply_path)
                except:
                    print(f"Failed to convert {obj_path}")