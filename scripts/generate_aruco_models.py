#!/usr/bin/env python3
"""
Download ArUco textures and generate simple Gazebo models for each marker.
Run this before launching Gazebo so model:// references resolve locally.
"""
import os
import urllib.request

MARKER_IDS = [70, 701, 702, 703, 705, 706, 708, 75]
BASE_RAW = 'https://raw.githubusercontent.com/AifosWhite/MCR2_Final_Challenge/karifm/worlds/aruco_textures'

PKG_ROOT = os.path.dirname(os.path.dirname(__file__))  # puzzlebot_sim2/
MODELS_DIR = os.path.join(PKG_ROOT, 'models')

os.makedirs(MODELS_DIR, exist_ok=True)

for mid in MARKER_IDS:
    name = f'aruco_{mid}'
    model_path = os.path.join(MODELS_DIR, name)
    textures_path = os.path.join(model_path, 'materials', 'textures')
    scripts_path = os.path.join(model_path, 'materials', 'scripts')
    os.makedirs(textures_path, exist_ok=True)
    os.makedirs(scripts_path, exist_ok=True)

    # Download texture
    url = f'{BASE_RAW}/aruco_{mid}.png'
    out_png = os.path.join(textures_path, f'aruco_{mid}.png')
    try:
        print('Downloading', url)
        urllib.request.urlretrieve(url, out_png)
    except Exception as e:
        print('Failed to download', url, e)
        continue

    # Write material script
    mat_script = os.path.join(scripts_path, 'aruco.material')
    mat_name = f'aruco/{mid}'
    with open(mat_script, 'w') as f:
        f.write(f"material {mat_name}\n{{\n  technique\n  {{\n    pass\n    {{\n      texture_unit\n      {{\n        texture aruco_{mid}.png\n      }}\n    }}\n  }}\n}}\n")

    # model.config
    model_config = f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>generated</name>
  </author>
  <description>ArUco marker {mid}</description>
</model>
'''
    with open(os.path.join(model_path, 'model.config'), 'w') as f:
        f.write(model_config)

    # model.sdf - simple vertical plane visual with material
    sdf = f'''<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <pose>0 0 0 0 -1.5708 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.18 0.18</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://puzzlebot_sim2/models/{name}/materials/scripts/aruco.material</uri>
            <name>aruco/{mid}</name>
          </script>
        </material>
      </visual>
      <collision name='collision'>
        <pose>0 0 0 0 -1.5708 0</pose>
        <geometry>
          <box>
            <size>0.18 0.02 0.18</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''
    with open(os.path.join(model_path, 'model.sdf'), 'w') as f:
        f.write(sdf)

print('Done generating models in', MODELS_DIR)
print('You can now run the script before launching Gazebo to make the markers available as model:// references.')
