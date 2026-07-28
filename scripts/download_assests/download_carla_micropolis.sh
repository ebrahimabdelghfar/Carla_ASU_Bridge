#!/bin/bash

# go to workspace root
cd "$(dirname "$0")/../.."

URL="https://micropolis626-my.sharepoint.com/:u:/g/personal/ebrahim_abdelghfar_micropolis_ai/IQDY-T7ui3gZRZiIPBkfqPZqAefqVBK-aLcgtrd3HcXGZGU?e=ayQe1D&download=1"
TAR_FILE="CarlaMicropolis.tar.gz"

echo "Download tar..."
wget -q --show-progress --progress=bar:force:noscroll -O "$TAR_FILE" "$URL"

echo "Extract tar to Carla_Micropolis..."
mkdir -p Carla_Micropolis
tar -xzf "$TAR_FILE" -C Carla_Micropolis

echo "Delete tar..."
rm "$TAR_FILE"

echo "Done."
