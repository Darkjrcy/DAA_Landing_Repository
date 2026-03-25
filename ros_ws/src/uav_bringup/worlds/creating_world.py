import bpy
import math
import os
import time
import urllib.request
from pathlib import Path

# Define the folder path worlds and meshes
ACT_FOLDER     = Path(__file__).resolve().parent 
SRC_FOLDER     = ACT_FOLDER.parent.parent
WORLDS_FOLDER  = (SRC_FOLDER / "uav_bringup" / "worlds") 
MODELS_FOLDER  = (SRC_FOLDER / "uav_bringup" / "models") 
MAPBOX_TOKEN   = ''  # ADD Mapbox token

# Funtion to get ht eworld limits:
def get_bounds(lat_cen, lon_cen, lat_mi, lon_mi):
    # Transofmration ratios:
    lat_deg_per_mi = 1 / 69.1
    lon_deg_per_mi = 1 / (69.1 * math.cos(math.radians(lat_cen)))

    # Calcualt eh half_lenghts at both directions:
    d_lat = (lat_mi / 2) * lat_deg_per_mi
    d_lon = (lon_mi / 2) * lon_deg_per_mi

    return (
        lat_cen - d_lat, 
        lat_cen + d_lat, 
        lon_cen - d_lon, 
        lon_cen + d_lon  
    )


# Clear the main scene from the cube and the camera:
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()


# DOwnload mapbox image:
def download_mapbox_image(min_lat, max_lat, min_lon, max_lon, save_path):
    lat_cen = (max_lat + min_lat) / 2
    lon_diff = max_lon - min_lon
    lat_diff = max_lat - min_lat
    aspect_ratio = (lon_diff * math.cos(math.radians(lat_cen))) / lat_diff
    
    if aspect_ratio >= 1:
        w, h = 1200, int(1200 / aspect_ratio)
    else:
        w, h = int(1200 * aspect_ratio), 1200
        
    url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/%5B{min_lon},{min_lat},{max_lon},{max_lat}%5D/{w}x{h}?access_token={MAPBOX_TOKEN}"
    try:
        urllib.request.urlretrieve(url, save_path)
        print("Image Download Successful!")
        return True
    except Exception as e:
        print(f"Image Download Failed: {e}")
        return False


# Function to import the osm file of a lat, lon coordiantes:
def import_osm_city(min_lat, max_lat, min_lon, max_lon, models_path, name):
    # Create the models folder:
    if not os.path.exists(models_path):
        os.makedirs(models_path)

    # Explicit tell wher the overaly images are saved:
    base_osm_path = os.path.join(str(Path.home()), "blender_plugins", "osm_files")
    if not os.path.exists(base_osm_path):
        os.makedirs(base_osm_path, exist_ok=True)

    # Clear the scene:
    clear_scene()
    
    # Access blosm:
    scene = bpy.context.scene
    blosm = scene.blosm

    # Lists of servers:
    servers = [
        "https://overpass.kumi.systems/api/interpreter",
        "https://overpass.nchc.org.tw/api/interpreter",
        "https://lz4.overpass-api.de/api/interpreter"
    ]

    # Access the addon preferences to change the server
    prefs = bpy.context.preferences.addons['blosm'].preferences

    # Set BLOSM preferences:
    prefs.osmDir = base_osm_path

    blosm.minLat, blosm.maxLat = min_lat, max_lat
    blosm.minLon, blosm.maxLon = min_lon, max_lon

    # Import the 3D mesh
    blosm.dataType = 'osm'        
    blosm.buildings = True       
    blosm.terrain = True
    for server in servers:
        prefs.overpassServer = server
        print(f"Trying Overpass server: {server}")
        try:
            bpy.ops.blosm.import_data()
            break
        except Exception as e:
            print(f"Server {server} failed or timed out. Trying next...")

    # Download the Image via Python 
    texture_path = os.path.join(str(models_path), "satellite.png")
    download_mapbox_image(min_lat, max_lat, min_lon, max_lon, texture_path)

    # Wait uintilt he image is downloaded:
    print("Checking for texture file...")
    timeout = 30
    start_time = time.time()
    while not os.path.exists(texture_path):
        if time.time() - start_time > timeout:
            print("ERROR: Texture download timed out!")
            break
        time.sleep(1)
        print("Waiting for file...")

    # Create and project the UVs:
    img = bpy.data.images.load(texture_path)
    mat = bpy.data.materials.new(name="TerrainMaterial")
    img.pack()
    mat.use_nodes = True

    # Setup Shader Nodes
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    tex_node = mat.node_tree.nodes.new('ShaderNodeTexImage')
    tex_node.image = img
    mat.node_tree.links.new(tex_node.outputs['Color'], bsdf.inputs['Base Color'])

    # Apply to the Terrain Object
    for obj in bpy.data.objects:
        if "terrain" in obj.name.lower():
            obj.data.materials.append(mat)
            if not obj.data.uv_layers:
                obj.data.uv_layers.new(name="UVMap")
            
            verts = obj.data.vertices
            min_x, max_x = min(v.co.x for v in verts), max(v.co.x for v in verts)
            min_y, max_y = min(v.co.y for v in verts), max(v.co.y for v in verts)
            
            for poly in obj.data.polygons:
                for loop_index in poly.loop_indices:
                    v_idx = obj.data.loops[loop_index].vertex_index
                    co = verts[v_idx].co
                    u = (co.x - min_x) / (max_x - min_x)
                    v = (co.y - min_y) / (max_y - min_y)
                    obj.data.uv_layers.active.data[loop_index].uv = (u, v)

    # Select all objects to export it:
    glb_file = os.path.join(str(models_path), f"{name}.glb")
    bpy.ops.file.pack_all()
    bpy.ops.object.select_all(action='SELECT')

    
    # Export the .glb file:
    try:
        bpy.ops.export_scene.gltf(
            filepath=glb_file, 
            export_format='GLB',
            export_yup=False,
            export_materials='EXPORT'
        )
        print("GLB Export successful! Textures are embedded.")
    except Exception as e:
        print(f"GLB Export failed: {e}")



# Function to geenrate the cofig file:
def generate_config_file(name, models_folder):
    # COnfig file basic strcutre:
    config_content = f"""<?xml version="1.0"?>
        <model>
            <name>{name}</name>
            <version>1.0</version>
            <sdf version="1.10">model.sdf</sdf>
            <author>
                <name>Blender Auto-Export</name>
                <email>auto@example.com</email>
            </author>
            <description>OSM Map of {name} generated from Blender.</description>
        </model>"""
    
    # Write the config file:
    config_path = os.path.join(models_folder, "model.config")
    with open(config_path, "w") as f:
        f.write(config_content)
    print(f"Created config: {config_path}")



# FUnction to create the sdf file:
def generate_sdf_file(name, models_folder):
    # SDF file basic strcutre:
    sdf_content = f"""<?xml version="1.0" ?>
        <sdf version="1.10">
            <model name="{name}">
                <static>true</static>
                <link name="city_link">
                <collision name="collision">
                    <geometry>
                    <mesh>
                        <uri>model://{name}/{name}.glb</uri>
                    </mesh>
                    </geometry>
                    <pose>0 0 0 0 0 0</pose>
                </collision>
                <visual name="visual">
                    <geometry>
                    <mesh>
                        <uri>model://{name}/{name}.glb</uri>
                    </mesh>
                    </geometry>
                    <pose>0 0 0 0 0 0</pose>
                </visual>
                </link>
            </model>
        </sdf>"""

    # write the sdf file:
    sdf_path = os.path.join(models_folder, "model.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_content)
    print(f"Created model SDF: {sdf_path}")



# Function to genreate the world file:
def generate_world_file(name, worlds_folder):
    # Basic world content:
    world_content = f"""<sdf version='1.10'>
        <world name='{name}'>
            <physics name='1ms' type='dart'>
            <max_step_size>0.005</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>0</real_time_update_rate>
            </physics>
            <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
            <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
            <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
            <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
            <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter" />
            <gravity>0 0 0</gravity>
            <magnetic_field>5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05</magnetic_field>
            <atmosphere type='adiabatic'/>
            <scene>
            <ambient>0.400000006 0.400000006 0.400000006 1</ambient>
            <background>0.699999988 0.699999988 0.699999988 1</background>
            <shadows>true</shadows>
            </scene>
            <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>
            <model name='{name}'>
            <pose>-0.91324664962335245 0.36039607076443403 0 0 0 0</pose>
            <static>true</static>
            <link name='city'>
                <visual name='visual'>
                <geometry>
                    <mesh>
                    <uri>model://{name}/{name}.glb</uri>
                    </mesh>
                </geometry>
                <pose>0 0 0 0 0 0</pose>
                </visual>
                <collision name='collision'>
                <geometry>
                    <mesh>
                    <uri>model://{name}/{name}.glb</uri>
                    </mesh>
                </geometry>
                <pose>0 0 0 0 0 0</pose>
                </collision>
                <pose>0 0 0 0 0 0</pose>
                <inertial>
                <pose>0 0 0 0 0 0</pose>
                <mass>1</mass>
                <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                </inertia>
                </inertial>
                <enable_wind>false</enable_wind>
            </link>
            <self_collide>false</self_collide>
            </model>
            <light name='sun' type='directional'>
            <pose>0 0 10 0 0 0</pose>
            <cast_shadows>true</cast_shadows>
            <intensity>1</intensity>
            <direction>-0.5 0.10000000000000001 -0.90000000000000002</direction>
            <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
            <specular>0.200000003 0.200000003 0.200000003 1</specular>
            <attenuation>
                <range>1000</range>
                <linear>0.01</linear>
                <constant>0.90000000000000002</constant>
                <quadratic>0.001</quadratic>
            </attenuation>
            <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
            </spot>
            </light>
        </world>
    </sdf>"""

    # Write teh world file:
    world_path = os.path.join(str(worlds_folder), f"{name}.world")
    with open(world_path, "w") as f:
        f.write(world_content)
    print(f"Created world file: {world_path}")




################################## MAIN CODE #######################################
# Define the characteristcs:
lon_cen = -78.64
lat_cen = -1.65335

# Define the area lengths in mi:
lon_length = 1.5
lat_length = 1

# Define the anme of the mesh file and world:
name = "Riobamba"
folder_name = (MODELS_FOLDER / name)



# Calculate the actual bounds
min_lat, max_lat, min_lon, max_lon = get_bounds(lat_cen, lon_cen, lat_length, lon_length)

# Run the import:
import_osm_city(min_lat, max_lat, min_lon, max_lon, folder_name, name)

# Run the functions to creat the gazebo mdodels file and world:
generate_config_file(name, folder_name)
generate_sdf_file(name, folder_name)
generate_world_file(name, WORLDS_FOLDER)



################################# RUN CODE #################################
# blender --background --python /home/jorge/DAA_Landing_Repository/ros_ws/src/uav_bringup/worlds/creating_world.py
############################################################################