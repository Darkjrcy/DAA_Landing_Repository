# Gazebo World Generator

A system to automatically generate Gazebo simulation worlds for specific geographic zones. This repository provides the `creating_world.py` script, which automates the process of generating terrain surfaces and building meshes from real-world coordinates.

## Objective
To simplify the creation of testing scenarios, this tool allows users to define a global zone and automatically generate a high-fidelity Gazebo world (SDF) including:
* **Surface Terrain:** Accurate elevation and satellite imagery.
* **Building Meshes:** 3D structures based on OpenStreetMap data.

## Prerequisites
The system relies on Blender as a processing engine and the **Blosm** plugin.

* **Blender:** Designed for **v4.12 LTS**.
* **Blosm Plugin:** [Download here](https://prochitecture.gumroad.com/l/blender-osm).
* **Mapbox API Token:** Required for satellite imagery. [Get one here](https://console.mapbox.com/account/access-tokens/).

---

## Installation & Setup

1. **Install Blender 4.12 LTS.**
2. **Install the Blosm plugin** within Blender (`Edit` > `Preferences` > `Add-ons`).
3. **Configure Blosm:** Enter your Mapbox token in the plugin settings to enable satellite textures.
4. **Package Configuration:** Create a `.yaml` file containing your Mapbox token in the `config` folder of the `auv_bringup` package.

---

## Usage
Run the generation script by calling Blender via the command line:

```bash
blender --python "/PATH/to/create_world.py"
```

---
## Configuration
Modify these variables within `create_world.py` to customize your world:

### Geographic Location
- `lat_cen` / `lon_cen`: Latitude and longitude of the world center.
- `lat_length` / `lon_length`: The dimensions of the area in miles.
### Model Quality
- **Blosm Version:** Set to `"free"` or `"pro"`.
    - `free`: Plain white buildings without textures.
    - `pro`: Realistic 3D buildings with textures and windows.

### General
- `name`: The name of the generated world. The SDF file will be saved automatically to the `world/` folder.
