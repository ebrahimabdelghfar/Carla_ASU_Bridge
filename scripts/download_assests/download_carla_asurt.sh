#!/bin/bash

# go to workspace root
cd "$(dirname "$0")/../.."

URL="https://engasuedu-my.sharepoint.com/:u:/g/personal/2301465_eng_asu_edu_eg/IQD4YytzmGiUSp_uHSNfLgpLAcBYgLsy1pdf0Ge9Z_d9QI8?e=DGcpwF&download=1"
TAR_FILE="ASU_RT_Carla.tar.gz"

echo "Download tar..."
wget -q --show-progress --progress=bar:force:noscroll -O "$TAR_FILE" "$URL"

echo "Extract tar to ASU_RT_Carla.tar.gz..."
mkdir -p ASU_RT_Carla
tar -xzf "$TAR_FILE" -C ASU_RT_Carla

echo "Delete tar..."
rm "$TAR_FILE"

echo "Done."
