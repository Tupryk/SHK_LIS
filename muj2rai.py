import os
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


def load_model_dicts(root, out_path: str, root_path: str) -> dict:
    models = {}
    meshes = root.findall(".//mesh")
    for mesh in meshes:
        name = mesh.attrib.get("name", "")
        path = mesh.attrib.get("file", "")
        path = os.path.join(root_path, path)

        ply_path = f"{name}.ply"
        try:
            ply_success = obj2ply(path, os.path.join(out_path, ply_path))
        except:
            ply_success = False
        if ply_success:
            models[name] = ply_path
        else:
            print(f"Error when tranforming {path} to ply")
            return None
    return models


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


if __name__ == "__main__":

    root_path = "fixtures"

    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file == "model.xml":
                file_path = os.path.join(root, file)
                tree = ET.parse(file_path)
                root_tree = tree.getroot()

                out_path = os.path.join(f"./rai_jointed/{root}")
                if not os.path.exists(out_path):
                    os.makedirs(out_path)

                models = load_model_dicts(root_tree, out_path, root)
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
                            try:
                                rai_joint = muj2rai_joint_map[joint_type]
                            except:
                                print(f"Joint type {joint_type} not implemented yet!")
                                rai_joint = "hingeX"
                                joint_limits = "-1, 1"

                            if not joint_limits:
                                joint_limits = "-1, 1"

                            line = f"""{body_name}_joint ({body_name}_joint_pre) {{joint: {rai_joint}, ctrl_limits: [{joint_limits}, 12]}}\n"""
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
                            if geom_type == "mesh":
                                mesh_name = geom.attrib.get("mesh", "")
                                line = f"""{body_name}_visual{i} ({body_name}) {{shape: mesh, mesh: "{models[mesh_name]}", contact: 0}}\n"""
                                lines.append(line)

                g_path = os.path.join(out_path, "./joint_scene.g")
                with open(g_path, "w") as file:
                    file.writelines(lines)
                
                print(f"Successfully created: {out_path}")
    
    remove_empty_subdirectories("rai_jointed")
