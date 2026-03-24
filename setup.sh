#!/bin/bash
echo "==== Downloading models.zip from Google Drive ===="

# Dowload and unzip the Models folder
# Create the folder (if it already exists, it's fine)
mkdir -p src/uav_bringup/models
# Create the rviz folder:
mkdir -p src/uav_description/rviz

# Google Drive file ID: 1612k0ajAOZc07ogzQaVJj58sSxcUIGob
MODELS_ID="1612k0ajAOZc07ogzQaVJj58sSxcUIGob"
gdown --id ${MODELS_ID} -O models.zip


echo "=== Unzipping the files into the models folder ==="
unzip -o models.zip -d src/uav_bringup/models/


# Delete the zip files:
echo "=== clean both zip files==="
rm models.zip
