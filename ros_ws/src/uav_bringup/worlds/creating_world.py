import bpy
import math

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



# Function to import the osm file of a lat, lon coordiantes:
def import_osm_city(min_lat, max_lat, min_lon, max_lon):
    # Access blosm:
    scene = bpy.context.scene
    blosm = scene.blosm

   # Access the addon preferences to change the server
    prefs = bpy.context.preferences.addons['blosm'].preferences
    prefs.overpassServer = "https://lz4.overpass-api.de/api/interpreter"

    # Set the map limits
    blosm.minLat, blosm.maxLat = min_lat, max_lat
    blosm.minLon, blosm.maxLon = min_lon, max_lon

    # Configure blosm settings
    blosm.dataType = 'osm'        
    blosm.buildings = True       
    blosm.terrain = True

    # Try the import again
    print(f"Attempting import from alternative server...")
    bpy.ops.blosm.import_data()







################################## MAIN CODE #######################################
# Define the characteristcs:
lon_cen = -78.64
lat_cen = -1.65335

# Define the area lengths in mi:
lon_length = 1.5
lat_length = 1

# Calculate the actual bounds
min_lat, max_lat, min_lon, max_lon = get_bounds(lat_cen, lon_cen, lat_length, lon_length)

# Run the import:
import_osm_city(min_lat, max_lat, min_lon, max_lon)

