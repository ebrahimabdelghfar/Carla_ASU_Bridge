#!/bin/bash
set -e  # Exit on any error

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Micropilot Sim - Isaac Sim Build${NC}"
echo -e "${GREEN}========================================${NC}"

WORKSPACE_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
REQ_FILE="${WORKSPACE_ROOT}/requirments.txt"
VENV_DIR="${WORKSPACE_ROOT}/isaac_env"
REQUIRED_PYTHON="python3.11"

echo -e "${YELLOW}Checking for ${REQUIRED_PYTHON}...${NC}"

# Function to check if python3.11 and venv exist
check_python_venv() {
    command -v ${REQUIRED_PYTHON} >/dev/null 2>&1 && ${REQUIRED_PYTHON} -m venv --help >/dev/null 2>&1
}

if ! check_python_venv; then
    echo -e "${RED}${REQUIRED_PYTHON} or python3.11-venv could not be found or is incomplete. Installing...${NC}"
    sudo apt-get update
    sudo apt-get install -y python3.11 python3.11-venv
else
    echo -e "${GREEN}${REQUIRED_PYTHON} and venv module are already installed.${NC}"
fi

echo -e "${YELLOW}Checking virtual environment in ${VENV_DIR}...${NC}"

if [ ! -d "${VENV_DIR}" ]; then
    echo -e "${YELLOW}Creating virtual environment...${NC}"
    ${REQUIRED_PYTHON} -m venv "${VENV_DIR}"
else
    echo -e "${GREEN}Virtual environment already exists.${NC}"
fi

echo -e "${YELLOW}Activating virtual environment...${NC}"
source "${VENV_DIR}/bin/activate"

# Verify that the correct python version is active
ACTIVE_PYTHON=$(python --version)
echo -e "${GREEN}Active Python version: ${ACTIVE_PYTHON}${NC}"

echo -e "${YELLOW}Installing dependencies from ${REQ_FILE}...${NC}"
if [ -f "${REQ_FILE}" ]; then
    pip install --upgrade pip
    pip install -r "${REQ_FILE}"
    echo -e "${GREEN}Dependencies installed successfully.${NC}"
else
    echo -e "${RED}Error: ${REQ_FILE} (requirements file) not found.${NC}"
    exit 1
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Isaac Sim setup completed successfully!${NC}"
echo -e "${GREEN}To activate the environment later, run: source isaac_env/bin/activate${NC}"
echo -e "${GREEN}========================================${NC}"
