import bpy
import math
import os
import time
import urllib.request
import urllib.parse
import mathutils
import json
from pathlib import Path

# Define the folder path worlds and meshes
ACT_FOLDER     = Path(__file__).resolve().parent 
SRC_FOLDER     = ACT_FOLDER.parent.parent
WORLDS_FOLDER  = (SRC_FOLDER / "uav_bringup" / "worlds") 
MODELS_FOLDER  = (SRC_FOLDER / "uav_bringup" / "models") 

# Add your MAPBOX TOKEN FROM TEH CONFIG FOLDER you have to created!!!:
mapbox_token_dir = (SRC_FOLDER / "uav_bringup" / "config" / "world_creation.yaml")
if os.path.exists(mapbox_token_dir):
    with open(mapbox_token_dir, 'r') as f:
        for line in f:
            if 'mapbox_token:' in line:
                MAPBOX_TOKEN = line.split(':')[-1].strip().replace("'", "").replace('"', "")
                break



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



# Function to ge the images from trees inside blender:
def get_image_from_tree(nodes):
    # First pass: check surface-level nodes
    for n in nodes:
        if n.type == 'TEX_IMAGE' and n.image:
            return n.image
    # Second pass: dig into Node Groups
    for n in nodes:
        if n.type == 'GROUP' and n.node_tree:
            found_img = get_image_from_tree(n.node_tree.nodes)
            if found_img:
                return found_img
    return None



# Clear the main scene from the cube and the camera:
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()



# DOwnload mapbox image:
def download_mapbox_image(img_min_lat, img_max_lat, img_min_lon, img_max_lon, save_path):
    # Incrase the image by 50% as it tooks some houses that are near the limits:
    lat_cen = (img_max_lat + img_min_lat) / 2
    lon_diff = img_max_lon - img_min_lon
    lat_diff = img_max_lat - img_min_lat
    aspect_ratio = (lon_diff * math.cos(math.radians(lat_cen))) / lat_diff
    
    # Make the aspect ratio the best resulition
    if aspect_ratio >= 1:
        w, h = 1280, int(1280 / aspect_ratio)
    else:
        w, h = int(1280 * aspect_ratio), 1280
        
    # DOwbnlaod the image from Mapbox:
    url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/%5B{img_min_lon},{img_min_lat},{img_max_lon},{img_max_lat}%5D/{w}x{h}@2x?access_token={MAPBOX_TOKEN}"
    try:
        urllib.request.urlretrieve(url, save_path)
        print("Image Download Successful!")
        return True
    except Exception as e:
        print(f"Image Download Failed: {e}")
        return False



# Download osm file:
def download_osm_file(min_lat, max_lat, min_lon, max_lon, save_path):
    bbox = f"{min_lon},{min_lat},{max_lon},{max_lat}"
    servers = [
        f"https://overpass-api.de/api/map?bbox={bbox}",
        f"https://lz4.overpass-api.de/api/map?bbox={bbox}",
        f"https://overpass.kumi.systems/api/map?bbox={bbox}",
        f"https://overpass.nchc.org.tw/api/map?bbox={bbox}"
    ]
    
    # Try to use each server to download the osm file inthe models folder:
    for url in servers:
        print(f"Trying server: {url}")
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            # GIev a 5 miniutes limit to sownload the osm file:
            with urllib.request.urlopen(req, timeout=300) as response, open(save_path, 'wb') as out_file:
                out_file.write(response.read())
            return True
        except Exception as e:
            time.sleep(2)
    return False



# Function to import the osm file of a lat, lon coordiantes:
def import_osm_city(min_lat, max_lat, min_lon, max_lon, models_path, name, blosm_ver):
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

    # Access the addon preferences to change the server
    prefs = bpy.context.preferences.addons['blosm'].preferences

    # ENABLE EXPERIMENTAL FEATURES:
    if hasattr(prefs, 'experimental'):
        prefs.experimental = True

    # Set BLOSM preferences:
    prefs.osmDir = base_osm_path
    prefs.mapboxAccessToken = MAPBOX_TOKEN

    blosm.minLat, blosm.maxLat = min_lat, max_lat
    blosm.minLon, blosm.maxLon = min_lon, max_lon
    # Increase image size and terrain to handle structues outside fot eh limsits:
    inc_perc = 0.5
    lat_pad = (max_lat - min_lat) * inc_perc
    lon_pad = (max_lon - min_lon) * inc_perc
    img_min_lat = min_lat - lat_pad
    img_max_lat = max_lat + lat_pad
    img_min_lon = min_lon - lon_pad
    img_max_lon = max_lon + lon_pad

    # IMport the terrain from osm first:
    blosm.minLat, blosm.maxLat = img_min_lat, img_max_lat
    blosm.minLon, blosm.maxLon = img_min_lon, img_max_lon
    blosm.dataType = 'terrain'
    try:
        bpy.ops.blosm.import_data()
    except Exception as e:
        print(f"Terrain import failed: {e}")

    # Apply the texture and download it to terrain:
    texture_path = os.path.join(str(models_path), "satellite.png")
    if download_mapbox_image(img_min_lat, img_max_lat, img_min_lon, img_max_lon, texture_path):
        print("Checking for texture file...")
        timeout = 30
        start_time = time.time()
        while not os.path.exists(texture_path):
            if time.time() - start_time > timeout:
                print("ERROR: Texture download timed out!")
                break
            time.sleep(1)
        img = bpy.data.images.load(texture_path)
        img.pack()
    else:
        print("Failed to download texture.")
        return

    # Find the terrain object Blosm just created
    terrain_obj = None
    for obj in bpy.context.scene.objects:
        if 'terrain' in obj.name.lower() and obj.type == 'MESH':
            terrain_obj = obj
            break

    if terrain_obj:
        terrain_obj.data.materials.clear() 
        
        mat = bpy.data.materials.new(name="TerrainMaterial")
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        
        bsdf = next(n for n in nodes if n.type == 'BSDF_PRINCIPLED')
        tex_node = nodes.new('ShaderNodeTexImage')
        tex_node.image = img
        links.new(tex_node.outputs['Color'], bsdf.inputs['Base Color'])
        
        terrain_obj.data.materials.append(mat)

        # Dynamic UV Projection based on the 3D mesh's actual world boundaries
        if not terrain_obj.data.uv_layers:
            terrain_obj.data.uv_layers.new(name="UVMap")
            
        matrix = terrain_obj.matrix_world
        verts = terrain_obj.data.vertices
        world_coords = [(matrix @ v.co) for v in verts]
        
        min_x, max_x = min(co.x for co in world_coords), max(co.x for co in world_coords)
        min_y, max_y = min(co.y for co in world_coords), max(co.y for co in world_coords)
        
        for poly in terrain_obj.data.polygons:
            for loop_index in poly.loop_indices:
                v_idx = terrain_obj.data.loops[loop_index].vertex_index
                co = matrix @ verts[v_idx].co 
                u = (co.x - min_x) / (max_x - min_x)
                v = (co.y - min_y) / (max_y - min_y)
                terrain_obj.data.uv_layers.active.data[loop_index].uv = (u, v)

    # Import the 3D mesh
    blosm.minLat, blosm.maxLat = min_lat, max_lat
    blosm.minLon, blosm.maxLon = min_lon, max_lon
    blosm.dataType = 'osm'            
    blosm.terrain = False


    # IMport the building in case you are in the pro version:
    if blosm_ver == "pro":
        blosm.buildings = True  

        # Force the 3D Realistic mode (handles roofs, complex geometry, and premium assets)
        if hasattr(blosm, 'buildingsType'):
            blosm.buildingsType = 'realistic'
        elif hasattr(blosm, 'buildingMode'):
            blosm.buildingMode = 'REALISTIC'
            
        # Enable the experimental texture export:
        if hasattr(blosm, 'importForExport'):
            blosm.importForExport = True
            
        # Ensure premium materials and roof generation are checked
        if hasattr(blosm, 'defaultMaterials'):
            blosm.defaultMaterials = True
        if hasattr(blosm, 'roofs'):
            blosm.roofs = True
    else:
        blosm.buildings = True

    # Try to download the .osm file:
    osm_file_path = os.path.join(models_path, f"{name}.osm")
    if not os.path.exists(osm_file_path):
        success = download_osm_file(min_lat, max_lat, min_lon, max_lon, osm_file_path)
        if not success:
            print("ERROR: Failed to download OSM file manually. Aborting building import.")
            return
    else:
        print("The OSM file already exists")

    # Create the 3D model of the city:
    blosm.osmSource = 'file'
    blosm.osmFilepath = str(osm_file_path)
    bpy.ops.blosm.import_data()

    # Clean all the extra things that are outside of the buildings:
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            name_lower = obj.name.lower()
            # If it's not the terrain and not a building, delete it
            if 'envelope' in name_lower:
                bpy.data.objects.remove(obj, do_unlink=True)
                continue
            if 'terrain' not in name_lower and 'buildings' not in name_lower:
                bpy.data.objects.remove(obj, do_unlink=True)

    # To avoid gazebo to crate terrrains on the 10 mi over the level of the sea move the terrain to 
    # sothe center altitude that you define:
    terrain_obj = None
    for obj in bpy.context.scene.objects:
       if 'terrain' in obj.name.lower() and obj.type == 'MESH':
            terrain_obj = obj
            break
    
    # FInd the lowest altitude
    if terrain_obj:
        matrix = terrain_obj.matrix_world
        verts = terrain_obj.data.vertices
        
        # Find the vertex closest to X=0, Y=0
        closest_z = 0.0
        min_dist = float('inf')
        
        for v in verts:
            world_co = matrix @ v.co
            # Calculate distance from origin on the XY plane (Pythagorean theorem)
            dist_to_center = (world_co.x ** 2) + (world_co.y ** 2)
            
            if dist_to_center < min_dist:
                min_dist = dist_to_center
                closest_z = world_co.z
                
        print(f"True altitude at (0,0) found: {closest_z} meters")

        # Shift all root objects down by that exact Z amount
        for obj in bpy.context.scene.objects:
            if obj.parent is None:  
                obj.location.z -= closest_z
                
        bpy.context.view_layer.update()
        print("World successfully snapped to Z = 0.")
    else:
        print("WARNING: Terrain object not found for Z-axis normalization.")

    # Change materials names so it doen't crash gazebo:
    for mat in bpy.data.materials:
        if getattr(mat, 'library', None):
            mat.make_local()
        # Change the name
        try:
            mat.name = mat.name.replace(".", "_").replace(" ", "_")
        except Exception as e:
            print(f"  [WARNING] Could not rename material {mat.name}: {e}")

    # Strip UV nmmaps and vertex colors:
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            # Remove all UV maps except our standard "UVMap"
            if obj.data.uv_layers:
                obj.data.uv_layers.active.name = "UVMap" 
                uvs_to_remove = [uv for uv in obj.data.uv_layers if uv.name != "UVMap"]
                for uv in uvs_to_remove:
                    obj.data.uv_layers.remove(uv)
                    
            # Remove all Vertex Colors except "Col"
            if hasattr(obj.data, 'color_attributes'):
                for vc in list(obj.data.color_attributes):
                    obj.data.color_attributes.remove(vc)

    # Select all objects to export it:
    gltf_file = os.path.join(str(models_path), f"{name}.gltf")
    bpy.ops.file.pack_all()
    bpy.ops.object.select_all(action='SELECT')

    # Clean image names and pack them inside Blender BEFORE export
    for img in bpy.data.images:
        if getattr(img, 'library', None):
            img.make_local()
            
        if img.name:
            clean_name = img.name.replace('\\', '/').split('/')[-1].replace(' ', '_')
            try:
                img.name = clean_name
                img.pack() 
            except Exception as e:
                print(f"  [WARNING] Could not rename/pack {img.name}: {e}")

    # Export the .gltf file:
    try:
        bpy.ops.export_scene.gltf(
            filepath=gltf_file, 
            export_format='GLTF_SEPARATE',
            export_yup=False,
            export_materials='EXPORT',
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
                        <uri>model://{name}/{name}.gltf</uri>
                    </mesh>
                    </geometry>
                    <pose>0 0 0 0 0 0</pose>
                </collision>
                <visual name="visual">
                    <geometry>
                    <mesh>
                        <uri>model://{name}/{name}.gltf</uri>
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
                    <uri>model://{name}/{name}.gltf</uri>
                    </mesh>
                </geometry>
                <pose>0 0 0 0 0 0</pose>
                </visual>
                <collision name='collision'>
                <geometry>
                    <mesh>
                    <uri>model://{name}/{name}.gltf</uri>
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

# Deifne teh plugin version (free or pro):
blosm_ver = "pro"

# Define the anme of the mesh file and world:
name = "Riobamba"
folder_name = (MODELS_FOLDER / name)



# Calculate the actual bounds
min_lat, max_lat, min_lon, max_lon = get_bounds(lat_cen, lon_cen, lat_length, lon_length)

# Run the import:
import_osm_city(min_lat, max_lat, min_lon, max_lon, folder_name, name, blosm_ver)

# Run the functions to creat the gazebo mdodels file and world:
generate_config_file(name, folder_name)
generate_sdf_file(name, folder_name)
generate_world_file(name, WORLDS_FOLDER)

# Close Blednder:
bpy.ops.wm.quit_blender()



################################# RUN CODE #################################
# blender --python "PATH_TO_CODE"
############################################################################
# INFO: This code is developed for Blender 4.2 LTS version.