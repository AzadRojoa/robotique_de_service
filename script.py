import os
import sys

def create_folder_structure(tag_number):
    base_folder = f"aruco_tag_{tag_number}"
    
    # Définition des sous-dossiers
    material_folder = os.path.join(base_folder, "materials", "textures")
    mesh_folder = os.path.join(base_folder, "meshes")
    
    # Création des dossiers
    os.makedirs(material_folder, exist_ok=True)
    os.makedirs(mesh_folder, exist_ok=True)

    # Contenu du fichier model.config
    model_config_content = f"""<?xml version="1.0" ?>
<model>
  <name>aruco_tag_{tag_number}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <description>
    A flat ARUco tag with the ID 110, designed to be used in Gazebo Harmonic.
    This model has a 1x1 m flat surface with a texture applied to it.
  </description>
  <author>
    <name>Yoan Mollard</name>
    <email>opensource@aubrune.eu</email>
  </author>
  <license>http://www.apache.org/licenses/LICENSE-2.0</license>
  <category>Other</category>
  <tags>ARUco, tag, marker, Gazebo</tags>
</model>"""

    # Création du fichier model.config dans le dossier de base
    model_config_path = os.path.join(base_folder, "model.config")
    with open(model_config_path, "w") as file:
        file.write(model_config_content)
        
    # Contenu du fichier model.sdf
    model_sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.7">
    <model name="aruco_tag_{tag_number}">
        <static>true</static>
        <pose>0 0 0 0 0 0</pose>
        <link name="link">
            <visual name="visual">
                <geometry>
                    <mesh>
                        <uri>model://aruco_tag_{tag_number}/meshes/aruco_tag_{tag_number}.obj</uri>
                        <scale>0.4 0.4 0.1</scale>
                    </mesh>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
"""

    # Création du fichier model.sdf dans le dossier de base
    model_sdf_path = os.path.join(base_folder, "model.sdf")
    with open(model_sdf_path, "w") as file:
        file.write(model_sdf_content)
    
    # Contenu du fichier aruco_tag_x.mtl
    aruco_tag_x_content_mlt = f"""newmtl aruco_tag_{tag_number}
map_Kd ../materials/textures/{tag_number}.jpg
map_Ka ../materials/textures/{tag_number}.jpg


"""

    # Création du fichier aruco_tag_x.mtl dans le dossier meshes
    aruco_tag_x_path_mlt = os.path.join(mesh_folder, f"aruco_tag_{tag_number}.mtl")
    with open(aruco_tag_x_path_mlt, "w") as file:
        file.write(aruco_tag_x_content_mlt)
    
    # Contenu du fichier aruco_tag_x.obj
    aruco_tag_x_content_obj = f"""mtllib aruco_tag_{tag_number}.mtl

# Vertices for a flat 1x1x0.001 box
v 0.0 0.0 0.0     # Vertex 1
v 1.0 0.0 0.0     # Vertex 2
v 1.0 1.0 0.0     # Vertex 3
v 0.0 1.0 0.0     # Vertex 4
v 0.0 0.0 0.001   # Vertex 5
v 1.0 0.0 0.001   # Vertex 6
v 1.0 1.0 0.001   # Vertex 7
v 0.0 1.0 0.001   # Vertex 8

# Vertex Normals (for each face of the box)
vn 0.0 0.0 -1.0   # Front face
vn 0.0 0.0 1.0    # Back face
vn -1.0 0.0 0.0   # Left face
vn 1.0 0.0 0.0    # Right face
vn 0.0 -1.0 0.0   # Bottom face
vn 0.0 1.0 0.0    # Top face

# Texture Coordinates for each face (simple mapping)
vt 0.0 0.0
vt 1.0 0.0
vt 1.0 1.0
vt 0.0 1.0

g FlatBox
usemtl aruco_tag_{tag_number}

# Faces of the flat box (12 triangles, 6 faces)
# Front face (2 triangles)
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1

# Back face (2 triangles)
f 5/1/2 6/2/2 7/3/2
f 5/1/2 7/3/2 8/4/2

# Left face (2 triangles)
f 1/1/3 2/2/3 6/3/3
f 1/1/3 6/3/3 5/4/3

# Right face (2 triangles)
f 4/1/4 3/2/4 7/3/4
f 4/1/4 7/3/4 8/4/4

# Bottom face (2 triangles)
f 1/1/5 4/2/5 8/3/5
f 1/1/5 8/3/5 5/4/5

# Top face (2 triangles)
f 2/1/6 3/2/6 7/3/6
f 2/1/6 7/3/6 6/4/6

# End of object
"""

    # Création du fichier aruco_tag_x.obj dans le dossier meshes
    aruco_tag_x_path_obj = os.path.join(mesh_folder, f"aruco_tag_{tag_number}.obj")
    with open(aruco_tag_x_path_obj, "w") as file:
        file.write(aruco_tag_x_content_obj)
    
    print(f"Structure créée pour '{base_folder}':")
    print(f"- {material_folder}")
    print(f"- {mesh_folder}")
    print(f"- {model_config_path} créé.")
    print(f"- {model_sdf_path} créé.")
    print(f"- {aruco_tag_x_path_mlt} créé.")
    print(f"- {aruco_tag_x_path_obj} créé.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <numéro>")
        sys.exit(1)
    
    tag_number = sys.argv[1]
    create_folder_structure(tag_number)
