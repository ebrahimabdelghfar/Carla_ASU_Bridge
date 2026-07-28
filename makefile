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
	@bash scripts/download_assests/download_carla_micropolis.sh
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
	$(CARLA_AS_USER) setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen -quality-level=Low & \
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
	$(CARLA_AS_USER) setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen & \
	until nc -z localhost 2000; do sleep 1; done; \
	setsid ros2 launch carla_telemetry_cpp carla_telemetry.launch.py auto_start:=$(AUTO_START) & \
	L_PID=$$!; \
	wait $$L_PID; \
	cleanup
launch_carla_sim_perf: kill_stale_sim
	@$(CARLA_AS_USER) setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen -quality-level=Low & \
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
# No kill_stale dep on purpose: that pkills every CarlaUE4 and would nuke siblings.
#   make launch_carla_servers CARLA_INSTANCES=3 CARLA_GPUS="0 1"
CARLA_INSTANCES ?= 1
CARLA_GPUS      ?= 0
CARLA_BASE_PORT ?= 2000
launch_carla_servers:
	@gpus=($(CARLA_GPUS)); ng=$${#gpus[@]}; n=$(CARLA_INSTANCES); p=$(CARLA_BASE_PORT); \
	declare -a used gpu; \
	for i in $$(seq 0 $$((n-1))); do \
		while nc -z localhost $$p 2>/dev/null || nc -z localhost $$((p+1)) 2>/dev/null || nc -z localhost $$((p+2)) 2>/dev/null; do \
			p=$$((p+10)); \
		done; \
		g=$${gpus[$$((i % ng))]}; \
		used[$$i]=$$p; gpu[$$i]=$$g; \
		echo -e "$(YELLOW)Starting CARLA server $$i on GPU $$g, RPC port $$p (log: /tmp/carla_server_$$p.log)$(NC)"; \
		$(CARLA_AS_USER) env CUDA_VISIBLE_DEVICES=$$g setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen \
			-graphicsadapter=$$g -carla-rpc-port=$$p >/tmp/carla_server_$$p.log 2>&1 & \
		p=$$((p+10)); \
	done; \
	fail=0; \
	for i in $${!used[@]}; do \
		q=$${used[$$i]}; \
		echo -e "$(YELLOW)Waiting for CARLA server $$i on :$$q...$(NC)"; \
		for t in $$(seq 1 120); do nc -z localhost $$q 2>/dev/null && break; sleep 1; done; \
		if nc -z localhost $$q 2>/dev/null; then \
			echo -e "$(GREEN)CARLA server $$i UP  ->  GPU $${gpu[$$i]}  RPC port $$q$(NC)"; \
		else \
			echo -e "$(RED)CARLA server $$i FAILED to open :$$q — see /tmp/carla_server_$$q.log$(NC)"; fail=1; \
		fi; \
	done; \
	echo -e "$(GREEN)Opened CARLA RPC ports: $${used[@]}$(NC)"; \
	exit $$fail
# make launch_carla_fleet CARLA_INSTANCES=3 CARLA_GPUS="0 1" DOMAIN_BASE=3
DOMAIN_BASE ?= 1
launch_carla_fleet:
	@source install/ros_apps/setup.bash; \
	gpus=($(CARLA_GPUS)); ng=$${#gpus[@]}; n=$(CARLA_INSTANCES); p=$(CARLA_BASE_PORT); \
	declare -a used gpu; \
	for i in $$(seq 0 $$((n-1))); do \
		while nc -z localhost $$p 2>/dev/null || nc -z localhost $$((p+1)) 2>/dev/null || nc -z localhost $$((p+2)) 2>/dev/null; do \
			p=$$((p+10)); \
		done; \
		g=$${gpus[$$((i % ng))]}; \
		used[$$i]=$$p; gpu[$$i]=$$g; \
		echo -e "$(YELLOW)Starting CARLA server $$i on GPU $$g, RPC port $$p (log: /tmp/carla_server_$$p.log)$(NC)"; \
		$(CARLA_AS_USER) env CUDA_VISIBLE_DEVICES=$$g setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen \
			-graphicsadapter=$$g -carla-rpc-port=$$p >/tmp/carla_server_$$p.log 2>&1 & \
		p=$$((p+10)); \
	done; \
	fail=0; \
	for i in $${!used[@]}; do \
		q=$${used[$$i]}; \
		echo -e "$(YELLOW)Waiting for CARLA server $$i on :$$q...$(NC)"; \
		for t in $$(seq 1 120); do nc -z localhost $$q 2>/dev/null && break; sleep 1; done; \
		if nc -z localhost $$q 2>/dev/null; then \
			echo -e "$(GREEN)CARLA server $$i UP  ->  GPU $${gpu[$$i]}  RPC port $$q$(NC)"; \
			d=$$(($(DOMAIN_BASE)+i)); tm=$$((8000+i)); cfg=/tmp/carla_config_$$q.yaml; \
			log=/tmp/carla_bridge_domain$$d.log; \
			sed -e "s|^\(\s*host:\).*|\1 \"localhost\"|" \
			    -e "s|^\(\s*port:\).*|\1 $$q|" \
			    -e "s|^\(\s*tm_port:\).*|\1 $$tm|" \
			    config/carla_interface_config.yaml > $$cfg; \
			echo -e "$(YELLOW)Launching bridge $$i -> :$$q  ROS_DOMAIN_ID=$$d  tm_port=$$tm (cfg: $$cfg, log: $$log)$(NC)"; \
			ROS_DOMAIN_ID=$$d setsid ros2 launch carla_telemetry_cpp carla_telemetry.launch.py \
				config_file:=$$cfg auto_start:=$(AUTO_START) >$$log 2>&1 & \
		else \
			echo -e "$(RED)CARLA server $$i FAILED to open :$$q — see /tmp/carla_server_$$q.log$(NC)"; fail=1; \
		fi; \
	done; \
	echo -e "$(GREEN)Opened CARLA RPC ports: $${used[@]}$(NC)"; \
	exit $$fail
launch_designer: kill_stale_sim
	@$(CARLA_AS_USER) setsid bash Carla_Micropolis/CarlaUE4.sh -vulkan -renderoffscreen & \
	C_PID=$$!; \
	cleanup() { \
		kill -9 -$$C_PID >/dev/null 2>&1 || true; \
		pkill -9 -f '[C]arlaUE4' >/dev/null 2>&1 || true; \
		for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done; \
	}; \
	trap 'cleanup; exit' INT TERM; \
	echo -e "${YELLOW}Waiting for CARLA server on :2000...${NC}"; \
	until nc -z localhost 2000; do sleep 1; done; \
	sleep 3; \
	echo -e "${GREEN}CARLA server up. Launching Scenario Designer...${NC}"; \
	cd $(WORKSPACE)/scenario_runner && python3 -m tools.scenario_designer.main --host localhost --port 2000; \
	cleanup
launch_designer_no_server:
	echo -e "${GREEN}Launching Scenario Designer...${NC}"; \
	cd $(WORKSPACE)/scenario_runner && python3 -m tools.scenario_designer.main --host localhost --port 2000; \
	cleanup
setup_ros2_workspace: format
	@bash scripts/ros_apps_build/colcon_build.sh
setup_docker:
	@if [ ! -d "Carla_Micropolis" ]; then \
		echo -e "${RED}Carla_Micropolis directory does not exist, please Run make download_carla_assets first${NC}"; \
		exit 1; \
	fi && \
	docker build -t upolis_carla_simulator:latest -f $(WORKSPACE)/docker/dockerfile $(WORKSPACE)
	$(DOCKER_COMPOSE) -f $(WORKSPACE)/docker/docker-compose.yml up -d
	@docker attach --no-stdin upolis_carla_simulator & \
	ATTACH_PID=$$! ; \
	docker exec upolis_carla_simulator /bin/bash -c 'until [ -f /micropilot/.initialized ]; do sleep 1; done' ; \
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
