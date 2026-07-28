#!/usr/bin/env bash
set -euo pipefail

# Resolve repository root from this script location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

VENV_ACTIVATE="${REPO_ROOT}/isaac_env/bin/activate"
SIM_SCRIPT="${REPO_ROOT}/scripts/operate_sim/assets/run_isaac_sim.py"

if [[ ! -f "${VENV_ACTIVATE}" ]]; then
	echo "[run_isaac_sim.sh] Missing virtual environment activate script: ${VENV_ACTIVATE}" >&2
	exit 1
fi

if [[ ! -f "${SIM_SCRIPT}" ]]; then
	echo "[run_isaac_sim.sh] Missing simulator entrypoint: ${SIM_SCRIPT}" >&2
	exit 1
fi

read -p "Do you want to set CPU to 'performance' mode? (requires sudo) [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
	echo "[run_isaac_sim.sh] Setting CPU scaling governor to performance..."
	for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
		if [ -f "$governor" ]; then
			echo "performance" | sudo tee "$governor" >/dev/null || echo "[run_isaac_sim.sh] Failed to set $governor"
		fi
	done
	echo "[run_isaac_sim.sh] CPU set to performance mode."
fi

# Activate Isaac Sim Python environment first.
source "${VENV_ACTIVATE}"

echo "[run_isaac_sim.sh] Activated isaac_env"
echo "[run_isaac_sim.sh] Running ${SIM_SCRIPT}"

exec python "${SIM_SCRIPT}"
