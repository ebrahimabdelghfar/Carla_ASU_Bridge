#!/usr/bin/env bash
set -e

cd /asurt
# Terminal Color
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

build_asurt() {
  echo -e "${GREEN}Building ROS 2 workspace in /asurt${NC}"
  make setup_ros2_workspace
  echo -e "${GREEN}Successfully built ROS 2 workspace${NC}"
}

if [ ! -f /asurt/.initialized ]; then
  build_asurt
  touch /asurt/.initialized
fi

exec "$@"