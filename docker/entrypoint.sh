#!/usr/bin/env bash
set -e

cd /asurt
# Terminal Color
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

build_asurt() {
  local target_dir="asurt"
  if [ -d "/asurt/$target_dir" ]; then
    echo -e "${GREEN}Building directory: /asurt/$target_dir${NC}"
    cd "/asurt/$target_dir"
    make setup_ros2_workspace
    cd /asurt
    echo -e "${GREEN}Successfully built directory: /asurt/$target_dir${NC}"
  else
    echo -e "${RED}Error: Directory /asurt/$target_dir does not exist.${NC}"
    return 1
  fi
}

if [ ! -f /asurt/.initialized ]; then
  build_asurt
  touch /asurt/.initialized
fi

exec "$@"