SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[1;33m
NC := \033[0m

AUTO_START?=false
GPUS?=all
export GPUS
DOCKER_COMPOSE ?= $(shell if docker compose version >/dev/null 2>&1; then echo "docker compose"; else echo "docker-compose"; fi)

ifneq (,$(wildcard /.dockerenv))
CARLA_AS_USER := sudo -u developer
else
CARLA_AS_USER :=
endif

CLANG_FORMAT ?= $(shell command -v clang-format-14 2>/dev/null || command -v clang-format 2>/dev/null)
FORMAT_DIRS ?= src/ros_apps/carla_telemetry_cpp
format:
	@if [ -z "$(CLANG_FORMAT)" ]; then \
		echo -e "$(RED)clang-format not found — sudo apt install clang-format-14$(NC)"; exit 1; \
	fi
	@find $(FORMAT_DIRS) \( -name '*.cpp' -o -name '*.hpp' \) -print0 | xargs -0 -r $(CLANG_FORMAT) -i
	@echo -e "$(GREEN)formatted $(FORMAT_DIRS) with $(notdir $(CLANG_FORMAT))$(NC)"
format_check:
	@if [ -z "$(CLANG_FORMAT)" ]; then \
		echo -e "$(RED)clang-format not found — sudo apt install clang-format-14$(NC)"; exit 1; \
	fi
	@find $(FORMAT_DIRS) \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
		| xargs -0 -r $(CLANG_FORMAT) --dry-run -Werror \
		&& echo -e "$(GREEN)format clean$(NC)"
install_git_hooks:
	@install -m 755 scripts/git_hooks/pre-commit "$$(git rev-parse --git-dir)/hooks/pre-commit"
	@echo -e "$(GREEN)pre-commit hook installed — staged C++ auto-formatted on commit$(NC)"

NET_BUF ?= 268435456
setup_cyclone:
	@echo -e "$(YELLOW)── Current CycloneDDS state ──────────────────────────$(NC)"
	@echo    "CYCLONEDDS_URI     = $${CYCLONEDDS_URI:-<unset>}"
	@echo    "RMW_IMPLEMENTATION = $${RMW_IMPLEMENTATION:-<unset>}"
	@printf  "cyclonedds.xml     = "; [ -f "$(HOME)/.config/cyclonedds/cyclonedds.xml" ] && echo "present" || echo -e "$(RED)MISSING$(NC)"
	@printf  "iox-roudi running  = "; pgrep -f iox-roudi >/dev/null 2>&1 && echo "yes" || echo -e "$(YELLOW)no$(NC)"
	@echo    "net.core.rmem_max  = $$(sysctl -n net.core.rmem_max)  (need >= $(NET_BUF))"
	@echo    "net.core.wmem_max  = $$(sysctl -n net.core.wmem_max)  (need >= $(NET_BUF))"
	@echo -e "$(YELLOW)── Raising kernel UDP socket buffers to $(NET_BUF) B ──$(NC)"
	@printf 'net.core.rmem_max=%s\nnet.core.wmem_max=%s\n' "$(NET_BUF)" "$(NET_BUF)" | sudo tee /etc/sysctl.d/60-cyclonedds.conf >/dev/null
	@sudo sysctl -q -w net.core.rmem_max=$(NET_BUF) net.core.wmem_max=$(NET_BUF)
	@echo -e "$(YELLOW)── Enabling multicast on all active interfaces ─────────$(NC)"
	@for iface in $$(ip -o link show up | awk -F': ' '{print $$2}' | cut -d'@' -f1); do \
		sudo ip link set $$iface multicast on || true; \
	done
	@echo -e "$(GREEN)  rmem_max/wmem_max now $$(sysctl -n net.core.rmem_max) B$(NC)"
	@echo -e "$(YELLOW)── Tuning cyclonedds.xml in place (parse + adjust) ───$(NC)"
	@python3 scripts/tune_cyclone_xml.py "$(HOME)/.config/cyclonedds/cyclonedds.xml" "120 MB"
	@echo -e "$(YELLOW)── RouDi mempools + daemon (xml left as tuned) ───────$(NC)"
	@bash scripts/setup_cyclonedds_shm.sh
	@echo -e "$(YELLOW)── Verify ────────────────────────────────────────────$(NC)"
	@grep -q '<Enable>true</Enable>' "$(HOME)/.config/cyclonedds/cyclonedds.xml" && echo -e "$(GREEN)SharedMemory enabled in xml$(NC)" || echo -e "$(RED)SharedMemory NOT enabled in xml$(NC)"
	@pgrep -f iox-roudi >/dev/null 2>&1 && echo -e "$(GREEN)iox-roudi ACTIVE$(NC)" || echo -e "$(RED)iox-roudi NOT running — check: journalctl --user -u iox-roudi.service$(NC)"
	@echo -e "$(GREEN)All set. Just run: make launch_carla_sim$(NC)"

BAK := bak-teardown.$(shell date +%s)
ETH_IFACE ?=
teardown_cyclone:
	@echo -e "$(YELLOW)── Stopping + removing iox-roudi service ─────────────$(NC)"
	@timeout 15 systemctl --user disable --now iox-roudi.service >/dev/null 2>&1 || true
	@rm -f "$(HOME)/.config/systemd/user/iox-roudi.service"
	@timeout 15 systemctl --user daemon-reload || true
	@timeout 15 systemctl --user reset-failed iox-roudi.service >/dev/null 2>&1 || true
	@pkill -TERM -x iox-roudi >/dev/null 2>&1 || true; sleep 1; pkill -KILL -x iox-roudi >/dev/null 2>&1 || true
	@echo -e "$(YELLOW)── Writing default CycloneDDS config (SHM off, wired NIC) ─$(NC)"
	@xml="$(HOME)/.config/cyclonedds/cyclonedds.xml"; \
	if [ -f "$$xml" ]; then cp "$$xml" "$$xml.$(BAK)"; echo -e "$(GREEN)  backed up old $$xml$(NC)"; fi
	@bash scripts/write_default_cyclone_xml.sh $(ETH_IFACE)
	@toml="$(HOME)/.config/iceoryx/roudi_config.toml"; \
	if [ -f "$$toml" ]; then mv "$$toml" "$$toml.$(BAK)"; echo -e "$(GREEN)  backed up + removed SHM $$toml$(NC)"; \
	else echo "  roudi_config.toml already absent"; fi
	@echo -e "$(YELLOW)── Kernel UDP buffers for 10MB-min default config ────$(NC)"
	@# The default xml requires SocketReceiveBufferSize min=10MB; node creation
	@# HARD-fails if net.core.rmem_max is below that. Drop the SHM-era 256MB
	@# drop-in and set a matching 26MB (drop-in for reboot + live now).
	@printf 'net.core.rmem_max=%s\nnet.core.wmem_max=%s\n' 26214400 26214400 | sudo tee /etc/sysctl.d/60-cyclonedds.conf >/dev/null
	@sudo sysctl -q -w net.core.rmem_max=26214400 net.core.wmem_max=26214400
	@echo -e "$(YELLOW)── Verify ────────────────────────────────────────────$(NC)"
	@pgrep -x iox-roudi >/dev/null 2>&1 && echo -e "$(RED)iox-roudi STILL running$(NC)" || echo -e "$(GREEN)iox-roudi stopped$(NC)"
	@xml="$(HOME)/.config/cyclonedds/cyclonedds.xml"; \
	grep -q '<SharedMemory>' "$$xml" && echo -e "$(RED)SharedMemory STILL in xml$(NC)" || echo -e "$(GREEN)default config written, SHM off$(NC)"; \
	echo -e "$(GREEN)  $$(grep -o 'NetworkInterface name=\"[^\"]*\"' "$$xml")$(NC)"
	@echo -e "$(GREEN)Done. CycloneDDS on plain UDP default, bound to wired NIC.$(NC)"
download_carla_assets:
	@bash scripts/download_assests/download_carla_asurt.sh
kill_stale_sim:
	@pkill -9 -f '[c]arla_telemetry_node' >/dev/null 2>&1 || true
	@# manual_control is a fork()ed child; when the bridge is SIGKILLed it is
	@# orphaned (reparented) and keeps streaming, flooding the next server.
	@pkill -9 -f '[c]arla_telemetry.manual_control' >/dev/null 2>&1 || true
	@pkill -f '[r]os2 launch carla_telemetry_cpp' >/dev/null 2>&1 || true
	@pkill -9 -f '[C]arlaUE4' >/dev/null 2>&1 || true
	@# Wait for the RPC port to be released so a fresh CARLA can bind it;
	@# starting a new server while the old one holds :2000 crashes it.
	@for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done
launch_carla_sim_low: kill_stale_sim
	@cleanup() { \
		trap '' INT TERM HUP; \
		for pat in '[C]arlaUE4' '[c]arla_telemetry_node' '[c]arla_telemetry.manual_control' '[s]tatic_transform_publisher' '[r]os2 launch carla_telemetry_cpp'; do \
			for p in $$(pgrep -f "$$pat"); do \
				[ "$$p" != "$$$$" ] && kill -9 "$$p" >/dev/null 2>&1 || true; \
			done; \
		done; \
		for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done; \
	}; \
	trap 'cleanup; exit' INT TERM HUP; \
	source install/ros_apps/setup.bash; \
	$(CARLA_AS_USER) setsid bash ASU_RT_Carla/CarlaUE4.sh -vulkan -renderoffscreen -quality-level=Low & \
	until nc -z localhost 2000; do sleep 1; done; \
	setsid ros2 launch carla_telemetry_cpp carla_telemetry.launch.py auto_start:=$(AUTO_START) & \
	L_PID=$$!; \
	wait $$L_PID; \
	cleanup
launch_carla_sim: kill_stale_sim
	@cleanup() { \
		trap '' INT TERM HUP; \
		for pat in '[C]arlaUE4' '[c]arla_telemetry_node' '[c]arla_telemetry.manual_control' '[s]tatic_transform_publisher' '[r]os2 launch carla_telemetry_cpp'; do \
			for p in $$(pgrep -f "$$pat"); do \
				[ "$$p" != "$$$$" ] && kill -9 "$$p" >/dev/null 2>&1 || true; \
			done; \
		done; \
		for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done; \
	}; \
	trap 'cleanup; exit' INT TERM HUP; \
	source install/ros_apps/setup.bash; \
	$(CARLA_AS_USER) DRI_PRIME=1 setsid bash ASU_RT_Carla/CarlaUE4.sh -vulkan -prefernvidia -renderoffscreen & \
	until nc -z localhost 2000; do sleep 1; done; \
	setsid ros2 launch carla_telemetry_cpp carla_telemetry.launch.py auto_start:=$(AUTO_START) & \
	L_PID=$$!; \
	wait $$L_PID; \
	cleanup
launch_carla_sim_perf: kill_stale_sim
	@$(CARLA_AS_USER) setsid bash ASU_RT_Carla/CarlaUE4.sh -vulkan -renderoffscreen -quality-level=Low & \
	C_PID=$$!; \
	source install/ros_apps/setup.bash; \
	cleanup() { \
		[ -n "$$L_PID" ] && kill -9 -$$L_PID >/dev/null 2>&1 || true; \
		pkill -9 -f '[c]arla_telemetry_node' >/dev/null 2>&1 || true; \
		pkill -9 -f '[c]arla_telemetry.manual_control' >/dev/null 2>&1 || true; \
		pkill -9 -f '[s]tatic_transform_publisher' >/dev/null 2>&1 || true; \
		pkill -9 -f '[r]os2 launch carla_telemetry_cpp' >/dev/null 2>&1 || true; \
		kill -9 -$$C_PID >/dev/null 2>&1 || true; \
		pkill -9 -f '[C]arlaUE4' >/dev/null 2>&1 || true; \
		for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done; \
	}; \
	trap 'cleanup; exit' INT TERM; \
	until nc -z localhost 2000; do sleep 1; done; \
	CARLA_PERF=1 setsid ros2 launch carla_telemetry_cpp carla_telemetry.launch.py auto_start:=$(AUTO_START) & \
	L_PID=$$!; \
	wait $$L_PID; \
	cleanup
launch_carla_sim_no_server:
	@source install/ros_apps/setup.bash && \
	ros2 launch carla_telemetry_cpp carla_telemetry.launch.py auto_start:=${AUTO_START};
run_dds_profiler:
	@source install/ros_apps/setup.bash && \
	ros2 run carla_telemetry_cpp dds_profiler_node \
		--ros-args -p config_file:=$(WORKSPACE)/config/carla_interface_config.yaml
run_sensor_sync_test:
	@source install/ros_apps/setup.bash && \
	ros2 run sensor_sync_test sensor_sync_test_node \
		--ros-args -p config_path:=$(WORKSPACE)/config/carla_interface_config.yaml
run_track_recorder:
	@source install/ros_apps/setup.bash && \
	ros2 launch track_recorder track_recorder.launch.py
visualize_track:
	@python3 src/ros_apps/track_recorder/scripts/visualize_track.py $(WORKSPACE)/src/ros_apps/global_racetrajectory_optimization/inputs/tracks/handling_track_recorded.csv
RACETRAJ_DIR := $(WORKSPACE)/src/ros_apps/global_racetrajectory_optimization
RACETRAJ_ENV ?= racetraj
RACETRAJ_TRACK ?= handling_track_recorded
RACETRAJ_OPT_TYPE ?= mincurv_iqp
setup_racetraj_env:
	@conda create -y -n $(RACETRAJ_ENV) python=3.7
	@echo -e "$(YELLOW)installing Cython<3 first — quadprog's build breaks against Cython>=3 (bug-005)$(NC)"
	@conda run -n $(RACETRAJ_ENV) pip install "Cython<3"
	@conda run -n $(RACETRAJ_ENV) pip install --no-build-isolation --no-cache-dir --no-deps quadprog==0.1.7
	@conda run -n $(RACETRAJ_ENV) pip install -r $(RACETRAJ_DIR)/requirements.txt
	@echo -e "$(GREEN)env '$(RACETRAJ_ENV)' ready — activate with: make activate_racetraj_env$(NC)"
activate_racetraj_env:
	@echo "run this in your shell (make cannot mutate your parent shell's env):"
	@echo "  conda activate $(RACETRAJ_ENV)"
generate_racetraj:
	@test -f "$(RACETRAJ_DIR)/inputs/tracks/$(RACETRAJ_TRACK).csv" || \
		{ echo -e "$(RED)track csv not found: $(RACETRAJ_DIR)/inputs/tracks/$(RACETRAJ_TRACK).csv$(NC)"; exit 1; }
	@cd $(RACETRAJ_DIR) && RACETRAJ_TRACK=$(RACETRAJ_TRACK) RACETRAJ_OPT_TYPE=$(RACETRAJ_OPT_TYPE) MPLBACKEND=Agg \
		conda run --no-capture-output -n $(RACETRAJ_ENV) python3 main_globaltraj.py
	@echo -e "$(GREEN)raceline written to $(RACETRAJ_DIR)/outputs/traj_race_cl.csv$(NC)"
visualize_racetraj:
	@conda run --no-capture-output -n $(RACETRAJ_ENV) python3 $(RACETRAJ_DIR)/visualize_racetraj.py \
		$(RACETRAJ_DIR)/outputs/traj_race_cl.csv \
		--track-csv $(RACETRAJ_DIR)/inputs/tracks/$(RACETRAJ_TRACK).csv
W_RIGHT ?= 4.0
W_LEFT ?= 4.0
change_track_width:
	@python3 src/ros_apps/track_recorder/scripts/change_track_width.py $(WORKSPACE)/src/ros_apps/global_racetrajectory_optimization/inputs/tracks/handling_track_recorded.csv $(W_RIGHT) $(W_LEFT)
setup_ros2_workspace: format
	@bash scripts/ros_apps_build/colcon_build.sh
setup_docker:
	@if [ ! -d "ASU_RT_Carla" ]; then \
		echo -e "${RED}ASU_RT_Carla directory does not exist, please Run make download_carla_assets first${NC}"; \
		exit 1; \
	fi && \
	docker build -t upolis_carla_simulator:latest -f $(WORKSPACE)/docker/dockerfile $(WORKSPACE)
	$(DOCKER_COMPOSE) -f $(WORKSPACE)/docker/docker-compose.yml up -d
	@docker attach --no-stdin asurt_carla_simulator & \
	ATTACH_PID=$$! ; \
	docker exec asurt_carla_simulator /bin/bash -c 'until [ -f /asurt/.initialized ]; do sleep 1; done' ; \
	EXIT_CODE=$$? ; \
	kill $$ATTACH_PID 2>/dev/null || true ; \
	exit $$EXIT_CODE
run_csv_scenarios:
	@python3 scenario_runner/run_csv_scenarios.py --csv scenario_runner/test_scenarios.csv --host localhost --port 2000
exec_docker_compose:
	@xhost +local:root 2>/dev/null || true
	$(DOCKER_COMPOSE) -f $(WORKSPACE)/docker/docker-compose.yml exec -e RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION) -it upolis_carla_simulator /bin/bash
down_docker_compose:
	@$(DOCKER_COMPOSE) -f $(WORKSPACE)/docker/docker-compose.yml down
