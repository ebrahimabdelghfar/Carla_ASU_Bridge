#!/usr/bin/env bash
set -e

cd /micropilot
# Terminal Color
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

build_asurt() {
  local target_dir="asurt"
  if [ -d "/micropilot/$target_dir" ]; then
    echo -e "${GREEN}Building directory: /micropilot/$target_dir${NC}"
    cd "/micropilot/$target_dir"
    make setup_ros2_workspace
    cd /micropilot
    echo -e "${GREEN}Successfully built directory: /micropilot/$target_dir${NC}"
  else
    echo -e "${RED}Error: Directory /micropilot/$target_dir does not exist.${NC}"
    return 1
  fi
}

if [ ! -f /micropilot/.initialized ]; then
  build_asurt
  touch /micropilot/.initialized
fi

exec "$@"