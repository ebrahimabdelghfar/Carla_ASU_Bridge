#!/bin/bash

set -e   # exit on first error

# ── Locate workspace root (two levels up from this script) ─────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "[build] Workspace root : $WS_ROOT"
echo "[build] Script dir     : $SCRIPT_DIR"

# ── Parse flags ────────────────────────────────────────────────────────────
SKIP_CARLA=false
EXTRA_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --no-carla) SKIP_CARLA=true ;;
        *)          EXTRA_ARGS+=("$arg") ;;
    esac
done

# ── 1. Source ROS 2 ────────────────────────────────────────────────────────
ROS_DISTROS=("humble" "jazzy" "iron" "rolling")
ROS_SOURCED=false
for distro in "${ROS_DISTROS[@]}"; do
    setup="/opt/ros/${distro}/setup.bash"
    if [ -f "$setup" ]; then
        # shellcheck source=/dev/null
        source "$setup"
        echo "[build] Sourced ROS 2 ${distro} from $setup"
        ROS_SOURCED=true
        break
    fi
done

if [ "$ROS_SOURCED" = false ]; then
    echo "[build] ERROR: No ROS 2 installation found under /opt/ros/."
    echo "        Install ROS 2 or source it manually before calling this script."
    exit 1
fi

# ── 2. CARLA Python API ────────────────────────────────────────────────────
if [ "$SKIP_CARLA" = false ]; then
    # Default path — sibling to the micropilot_sim workspace
    CARLA_WHEEL_DIR="$HOME/carla/PythonAPI/carla/dist"

    if python3 -c "import carla" &>/dev/null; then
        echo "[build] carla Python module already importable — skipping install."
    elif [ -d "$CARLA_WHEEL_DIR" ]; then
        WHEEL=$(ls "$CARLA_WHEEL_DIR"/carla-*.whl 2>/dev/null | head -1)
        if [ -n "$WHEEL" ]; then
            echo "[build] Installing CARLA wheel: $WHEEL"
            pip3 install --quiet "$WHEEL"
        else
            echo "[build] WARNING: No CARLA .whl found in $CARLA_WHEEL_DIR"
        fi
    else
        echo "[build] WARNING: CARLA wheel directory not found at $CARLA_WHEEL_DIR"
        echo "        Set CARLA_WHEEL_DIR or use --no-carla to skip."
    fi

    # Fallback: prepend egg directory if pip install was not done
    CARLA_EGG_DIR="$HOME/carla/PythonAPI/carla"
    if [ -d "$CARLA_EGG_DIR" ] && ! python3 -c "import carla" &>/dev/null; then
        echo "[build] Adding CARLA PythonAPI to PYTHONPATH (egg fallback)."
        export PYTHONPATH="$CARLA_EGG_DIR:${PYTHONPATH:-}"
    fi
fi

SETUPTOOLS_VERSION=$(python3 -c "import setuptools; print(setuptools.__version__)" 2>/dev/null)
SETUPTOOLS_MAJOR=$(echo "$SETUPTOOLS_VERSION" | cut -d'.' -f1)

if [ -n "$SETUPTOOLS_MAJOR" ] && [ "$SETUPTOOLS_MAJOR" -ge 64 ]; then
    echo "[build] Detected setuptools $SETUPTOOLS_VERSION (>= 64)."
    echo "[build] Downgrading to setuptools<64 for colcon-python-setup-py compatibility..."
    pip3 install --quiet "setuptools<64"
    NEW_VER=$(python3 -c "import setuptools; print(setuptools.__version__)" 2>/dev/null)
    echo "[build] setuptools downgraded to $NEW_VER"
else
    echo "[build] setuptools $SETUPTOOLS_VERSION is compatible — no downgrade needed."
fi

export COLCON_DEFAULTS_FILE="${SCRIPT_DIR}/config_colcon.yaml"
export COLCON_LOG_PATH="${WS_ROOT}/logs/build_logs/ros_apps_build"

mkdir -p "$COLCON_LOG_PATH"

echo "[build] COLCON_DEFAULTS_FILE : $COLCON_DEFAULTS_FILE"
echo "[build] COLCON_LOG_PATH      : $COLCON_LOG_PATH"

cd "$SCRIPT_DIR"   # colcon must run from the directory containing colcon.yaml

echo ""
echo "[build] Running: colcon build --symlink-install ${EXTRA_ARGS[*]}"
echo "────────────────────────────────────────────────────────────────"

colcon build --symlink-install "${EXTRA_ARGS[@]}"