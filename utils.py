import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import numpy as np

def pose_matrix_to_7d(matrix):
    """
    Converts a 4x4 pose matrix (rotation + translation) into a 7D pose vector (translation + quaternion).
    
    Args:
        matrix (numpy.ndarray): 4x4 transformation matrix.
    
    Returns:
        numpy.ndarray: 7D pose vector [tx, ty, tz, qw, qx, qy, qz].
    """
    assert matrix.shape == (4, 4), "Input must be a 4x4 matrix"
    
    translation = matrix[:3, 3]  # Extract translation (tx, ty, tz)
    rotation_matrix = matrix[:3, :3]  # Extract 3x3 rotation matrix
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()  # Convert to quaternion (qx, qy, qz, qw)
    quaternion = np.roll(quaternion, shift=1)  # Move qw to the first position
    
    return np.hstack((translation, quaternion))


def get_texture_file(xml_file, geom_name):
    """
    Get the texture file corresponding to a given geom name in a MuJoCo XML file.

    Parameters:
    - xml_file (str): Path to the MuJoCo XML file.
    - geom_name (str): Name of the geom.

    Returns:
    - str: The texture file path if found, otherwise None.
    """
    # Parse the XML file
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Find the geom with the given name
    geom = root.find(f".//geom[@name='{geom_name}']")
    if geom is None:
        raise ValueError(f"No geom with name '{geom_name}' found in the XML file.")

    # Get the material name from the geom
    material_name = geom.get("material")
    if not material_name:
        raise ValueError(f"The geom '{geom_name}' does not have an associated material.")

    # Find the material with the given name
    material = root.find(f".//material[@name='{material_name}']")
    if material is None:
        raise ValueError(f"No material with name '{material_name}' found in the XML file.")

    # Get the texture name from the material
    texture_name = material.get("texture")
    if not texture_name:
        raise ValueError(f"The material '{material_name}' does not have an associated texture.")

    # Find the texture with the given name
    texture = root.find(f".//texture[@name='{texture_name}']")
    if texture is None:
        raise ValueError(f"No texture with name '{texture_name}' found in the XML file.")

    # Get the texture file path
    texture_file = texture.get("file")
    if not texture_file:
        raise ValueError(f"The texture '{texture_name}' does not have an associated file.")

    return texture_file

if __name__ == "__main__":
    # Example usage
    xml_file_path = "model.xml"
    geom_name = "knob_rear_center_main"

    try:
        texture_file = get_texture_file(xml_file_path, geom_name)
        print(f"The texture file for geom '{geom_name}' is: {texture_file}")
    except ValueError as e:
        print(e)
