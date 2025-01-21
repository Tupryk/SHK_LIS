import os
import rowan
import numpy as np
import xml.etree.ElementTree as ET

from robocasa2rai import obj2ply


muj2rai_joint_map = {
    "1 0 0": "hingeX",
    "0 1 0": "hingeY",
    "0 0 1": "hingeZ",
}


def invert_floats(input_string):
    numbers = input_string.split()
    inverted_numbers = [str(-1 * float(num)) for num in numbers]
    output_string = " ".join(inverted_numbers)
    return output_string


def load_model_dicts(root, out_path: str, root_path: str, materials: dict[str]) -> dict:
    models = {}
    meshes = root.findall(".//mesh")
    for mesh in meshes:
        name = mesh.attrib.get("name", "")
        path = mesh.attrib.get("file", "")
        material_name = mesh.attrib.get("material", "")
        path = os.path.join(root_path, path)
        texture_path = materials[material_name]

        ply_path = f"{name}.ply"
        # try:
        ply_success = obj2ply(path, os.path.join(out_path, ply_path), texture_path=texture_path)
        # except Exception as e:
            # ply_success = False
            # print(e)
        if ply_success:
            models[name] = ply_path
        else:
            print(f"Error when tranforming {path} to ply")
            return None
        
    return models


def get_ply_path(name: str, obj_path: str, texture_path: str, root_path: str, out_path: str) -> str:

    obj_path = os.path.join(root_path, obj_path)
    texture_path = os.path.join(root_path, texture_path)
    ply_path = f"{name}.ply"

    ply_success = obj2ply(obj_path, os.path.join(out_path, ply_path), texture_path=texture_path)
    if ply_success:
        return ply_path
    return None


def find_parent_body(child_name, root_tree):
    for parent_body in root_tree.findall(".//body"):
        child_body = parent_body.find(f"./body[@name='{child_name}']")
        if child_body is not None:
            parent_body = parent_body.attrib.get("name", "")
            if parent_body:
                return parent_body
            return None

    return None



def remove_empty_subdirectories(folder_path):
    for root, dirs, files in os.walk(folder_path, topdown=False):
        for directory in dirs:
            dir_path = os.path.join(root, directory)
            if not os.listdir(dir_path):
                os.rmdir(dir_path)
                print(f"Removed empty directory: {dir_path}")


def relative_rotation(vec1, vec2):
    # Normalize the input vectors
    v1 = vec1 / np.linalg.norm(vec1)
    v2 = vec2 / np.linalg.norm(vec2)
    
    # Compute the cross product and the dot product
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    
    # Handle edge cases
    if np.allclose(v1, v2):
        # Vectors are aligned; no rotation needed
        return np.array([1, 0, 0, 0])  # Identity quaternion
    elif np.allclose(v1, -v2):
        # Vectors are opposite; 180-degree rotation
        # Choose an arbitrary orthogonal vector for the axis
        axis = np.array([1, 0, 0]) if not np.allclose(v1, [1, 0, 0]) else np.array([0, 1, 0])
        return rowan.from_axis_angle(axis, np.pi)
    
    # Compute the quaternion
    q = np.array([1 + dot, *cross])
    return rowan.normalize(q)


def get_models_mats(root_tree, root_path: str):
    materials = {}
    textures = {}

    texs = root_tree.findall(".//texture")
    for tex in texs:
        tex_name = tex.attrib.get("name", "")
        file_path = tex.attrib.get("file", "")
        textures[tex_name] = file_path

    maters = root_tree.findall(".//material")
    for mater in maters:
        mater_name = mater.attrib.get("name", "")
        tex_name = mater.attrib.get("texture", "")
        if tex_name:
            materials[mater_name] = textures[tex_name]
        else:
            materials[mater_name] = ""

    models = {}
    meshes = root_tree.findall(".//mesh")
    for mesh in meshes:
        name = mesh.attrib.get("name", "")
        path = mesh.attrib.get("file", "")
        models[name] = path

    return models, materials


if __name__ == "__main__":

    root_path = "fixtures/stoves/coil_burners_induc"

    total = 0
    successful = 0
    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file == "model.xml":
                file_path = os.path.join(root, file)
                tree = ET.parse(file_path)
                root_tree = tree.getroot()

                out_path = os.path.join(f"./rai_jointed/{root}")
                if not os.path.exists(out_path):
                    os.makedirs(out_path)

                # Get texture and model paths
                models, materials = get_models_mats(root_tree, root_path)
                if models is None:
                    continue

                lines = []
                bodies = root_tree.findall(".//body")
                for body in bodies:
                    body_name = body.attrib.get("name", "")

                    if body_name:

                        joints = body.findall("./joint")
                        parent_body = find_parent_body(body_name, root_tree)
                        
                        if len(joints):
                            joint_type = joints[0].attrib.get("axis", "")
                            joint_pos = joints[0].attrib.get("pos", "")
                            joint_limits = joints[0].attrib.get("range", "").replace(" ", ", ")
                            line = f"""{body_name}_joint_pre ({parent_body}) {{Q: "t({joint_pos})"}}\n"""
                            lines.append(line)
                            
                            pre = f"{body_name}_joint_pre"
                            splited_joint_params = joint_type.split()
                            if len(splited_joint_params) == 3:
                                if joint_type in muj2rai_joint_map.keys():
                                    rai_joint = muj2rai_joint_map[joint_type]
                                else:
                                    vec1 = np.array([0., 0., 1.])
                                    vec2 = np.array([
                                        float(splited_joint_params[0]),
                                        float(splited_joint_params[1]),
                                        float(splited_joint_params[2])
                                    ])
                                    quat = relative_rotation(vec1, vec2)
                                    new_pre = f"{pre}_special_joint"
                                    line = f"""{new_pre} ({pre}) {{quaternion: [{quat[0]}, {quat[1]}, {quat[2]}, {quat[3]}]}}\n"""
                                    pre = new_pre
                                    lines.append(line)
                                    rai_joint = "hingeZ"
                            else:
                                print(f"Joint type {joint_type} not implemented yet!")
                                rai_joint = "hingeX"
                                joint_limits = "-1, 1"

                            if not joint_limits:
                                joint_limits = "-1, 1"

                            line = f"""{body_name}_joint ({pre}) {{joint: {rai_joint}, ctrl_limits: [{joint_limits}, 12]}}\n"""
                            lines.append(line)
                            line = f"""{body_name} ({body_name}_joint) {{Q: "t({invert_floats(joint_pos)})"}}\n"""

                        elif parent_body:
                            line = f"""{body_name} ({parent_body}) {{}}\n"""

                        else:
                            line = f"""{body_name} {{}}\n"""

                        lines.append(line)

                        geoms = body.findall("./geom")
                        for i, geom in enumerate(geoms):
                            
                            geom_type = geom.attrib.get("type", "")
                            geom_name = geom.attrib.get("name", "")
                            if geom_type == "mesh":
                                
                                mesh_name = geom.attrib.get("mesh", "")
                                material_name = geom.attrib.get("material", "")
                                
                                ply_path = get_ply_path(f"{body_name}_geom", models[mesh_name], materials[material_name], root_path, out_path)
                                
                                line = f"""{body_name}_visual{i} ({body_name}) {{shape: mesh, mesh: "{ply_path}", contact: 0}}\n"""
                                lines.append(line)

                g_path = os.path.join(out_path, "./joint_scene.g")
                with open(g_path, "w") as file:
                    file.writelines(lines)
                
                print(f"Successfully created: {out_path}")
    
    remove_empty_subdirectories("rai_jointed")
