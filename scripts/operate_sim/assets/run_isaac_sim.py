"""
run_isaac_sim.py — Launch script for Micropolis.Telemetry in Isaac Sim 5.1.

The simulator does NOT start automatically.
Control it via ROS 2 services:
    ros2 service call /micropolis/sim/start std_srvs/srv/Trigger "{}"
    ros2 service call /micropolis/sim/stop  std_srvs/srv/Trigger "{}"
"""

import sys
import os
from pathlib import Path

sys.path = [p for p in sys.path if "/opt/ros" not in p]

REPO_ROOT = Path(__file__).resolve().parents[3]
DATA_EXT_FOLDER = REPO_ROOT / "data"

# Keep runtime behavior stable for modules/configs that use relative paths.
os.chdir(REPO_ROOT)

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "disable_viewport_updates": True,
    "extra_args": [
        "--ext-folder", str(DATA_EXT_FOLDER),
        "--enable", "Micropolis.Telemetry",
        "--/rtx/post/dlss/execMode=0",          
        "--/rtx/ecoMode/maxFramesWithoutChange=60",
        "--/rtx/ecoMode/active=true"  
    ],
})

print("[run_isaac_sim] SimulationApp ready. Initialising telemetry...")

import queue
import threading
import traceback
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Empty

import omni.timeline
import omni.usd
from pxr import UsdPhysics

from omni.isaac.core.world import World

from Micropolis.Telemetry.global_variables import CONFIG_FILE, DEFAULT_WORLD_SETTINGS
from Micropolis.Telemetry.ui.ui_window import TelemetryWindow
from Micropolis.Telemetry.ui.ui_delegate import UIDelegate


# ──────────────────────────────────────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────────────────────────────────────
try:
    with open(CONFIG_FILE) as f:
        cfg = yaml.safe_load(f) or {}
    print(f"[run_isaac_sim] Config loaded from {CONFIG_FILE}")
except Exception as e:
    print(f"[run_isaac_sim] Config load failed ({e}), using defaults.")
    cfg = {}


# ──────────────────────────────────────────────────────────────────────────────
# UI
# ──────────────────────────────────────────────────────────────────────────────
try:
    delegate = UIDelegate(cfg)
    window = TelemetryWindow(delegate)
    print("[run_isaac_sim] UI window created.")
except Exception as e:
    print(f"[run_isaac_sim] UI creation failed: {e}")
    traceback.print_exc()
    delegate = None


# ──────────────────────────────────────────────────────────────────────────────
# World
# ──────────────────────────────────────────────────────────────────────────────
try:
    if World.instance() is None:
        world = World(**DEFAULT_WORLD_SETTINGS)
        print("[run_isaac_sim] World created.")
    else:
        world = World.instance()
        print("[run_isaac_sim] Reusing existing World.")
except Exception as e:
    print(f"[run_isaac_sim] World init failed: {e}")
    traceback.print_exc()
    world = World.instance()

# Enable GPU dynamics via physics context
try:
    physics_context = world.get_physics_context()
    physics_context.enable_gpu_dynamics(True)
    print("[run_isaac_sim] GPU dynamics enabled via physics context.")
except Exception as e:
    print(f"[run_isaac_sim][WARN] Failed to enable GPU dynamics: {e}")


# ──────────────────────────────────────────────────────────────────────────────
# Timeline
# ──────────────────────────────────────────────────────────────────────────────
_tl = omni.timeline.get_timeline_interface()

try:
    if _tl.is_playing():
        _tl.stop()
        simulation_app.update()
except Exception as e:
    print(f"[run_isaac_sim] Initial timeline pause warning: {e}")


# ──────────────────────────────────────────────────────────────────────────────
# Debug helpers
# ──────────────────────────────────────────────────────────────────────────────
def _list_physics_scenes():
    """Print all physics scenes in the current USD stage."""
    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[run_isaac_sim][DEBUG] No USD stage available yet.")
            return []

        scenes = []
        for prim in stage.Traverse():
            try:
                if prim.IsA(UsdPhysics.Scene):
                    scenes.append(str(prim.GetPath()))
            except Exception:
                pass

        print(f"[run_isaac_sim][DEBUG] Physics scenes found: {scenes}")
        if len(scenes) > 1:
            print("[run_isaac_sim][WARNING] More than one PhysicsScene found. "
                  "Your previous error strongly suggests multi-scene/per-scene-step issues.")
        return scenes
    except Exception as e:
        print(f"[run_isaac_sim][DEBUG] Failed to inspect physics scenes: {e}")
        return []


def _print_timeline_state(prefix: str):
    try:
        print(f"[run_isaac_sim][DEBUG] {prefix}: timeline.is_playing()={_tl.is_playing()}")
    except Exception as e:
        print(f"[run_isaac_sim][DEBUG] {prefix}: failed to query timeline state: {e}")


_list_physics_scenes()
_print_timeline_state("startup")


# ──────────────────────────────────────────────────────────────────────────────
# Sim state
# ──────────────────────────────────────────────────────────────────────────────
_sim_control_queue = queue.SimpleQueue()
_desired_running = False
_last_applied_running = None
_sim_state_lock = threading.Lock()


def _set_desired_running(desired: bool):
    global _desired_running
    with _sim_state_lock:
        _desired_running = desired


def _get_desired_running() -> bool:
    with _sim_state_lock:
        return _desired_running


def _hard_pause():
    """
    Force pause via timeline only.
    Isaac Sim timeline control is exposed through get_timeline_interface().pause().
    """
    try:
        _tl.stop()
        simulation_app.update()
        _print_timeline_state("after STOP")
    except Exception as e:
        print(f"[run_isaac_sim][ERROR] hard pause failed: {e}")
        traceback.print_exc()


def _hard_play():
    """
    Force play via timeline only.
    """
    try:
        _tl.play()
        simulation_app.update()
        _print_timeline_state("after START")
    except Exception as e:
        print(f"[run_isaac_sim][ERROR] hard play failed: {e}")
        traceback.print_exc()


def _timeline_is_playing():
    """
    Best-effort timeline state query.
    Returns True/False, or None if state cannot be read.
    """
    try:
        return bool(_tl.is_playing())
    except Exception as e:
        print(f"[run_isaac_sim][WARN] Failed to query timeline state: {e}")
        return None


def _drain_sim_requests():
    """
    Drain queue and coalesce commands.
    STOP wins over START if both are received in the same frame.
    """
    saw_start = 0
    saw_stop = 0
    while True:
        try:
            try:
                cmd = _sim_control_queue.get_nowait()
            except AttributeError:
                cmd = _sim_control_queue.get(block=False)

            if cmd == "start":
                saw_start += 1
            elif cmd == "stop":
                saw_stop += 1
        except queue.Empty:
            break

    if saw_stop > 0:
        if saw_start > 0:
            print(
                "[run_isaac_sim] START/STOP burst received in same frame; "
                "prioritising STOP."
            )
        _set_desired_running(False)
        print(f"[run_isaac_sim] STOP requested ({saw_stop} event(s)).")
    elif saw_start > 0:
        _set_desired_running(True)
        print(f"[run_isaac_sim] START requested ({saw_start} event(s)).")


def _apply_sim_state_if_needed():
    global _last_applied_running

    desired = _get_desired_running()
    actual = _timeline_is_playing()

    # Apply on command changes and also when the real timeline state drifts
    # from the desired state (e.g. another callback resumes playback).
    needs_apply = desired != _last_applied_running
    if actual is not None and actual != desired:
        needs_apply = True

    if not needs_apply:
        return

    if desired:
        print("[run_isaac_sim] Applying START...")
        _hard_play()
    else:
        print("[run_isaac_sim] Applying STOP...")
        _hard_pause()

    _last_applied_running = desired


# ──────────────────────────────────────────────────────────────────────────────
# ROS node
# ──────────────────────────────────────────────────────────────────────────────
class SimControlNode(Node):
    """
    Services:
      /micropolis/sim/start  (std_srvs/Trigger) — plays the timeline
      /micropolis/sim/stop   (std_srvs/Trigger) — pauses the timeline

    Legacy topics:
      /micropolis/sim/start  (std_msgs/Empty)
      /micropolis/sim/stop   (std_msgs/Empty)
    """
    def __init__(self):
        super().__init__("micropolis_sim_control")

        self.create_service(Trigger, "/micropolis/sim/start", self._on_start_service)
        self.create_service(Trigger, "/micropolis/sim/stop", self._on_stop_service)

        self.create_subscription(Empty, "/micropolis/sim/start", self._on_start_topic, 1)
        self.create_subscription(Empty, "/micropolis/sim/stop", self._on_stop_topic, 1)

        self.get_logger().info(
            "SimControlNode ready.\n"
            "  Start: ros2 service call /micropolis/sim/start std_srvs/srv/Trigger '{}'\n"
            "  Stop:  ros2 service call /micropolis/sim/stop  std_srvs/srv/Trigger '{}'\n"
            "  Legacy start topic: ros2 topic pub --once /micropolis/sim/start std_msgs/msg/Empty '{}'\n"
            "  Legacy stop topic:  ros2 topic pub --once /micropolis/sim/stop  std_msgs/msg/Empty '{}'"
        )

    def _queue_start(self, source: str):
        self.get_logger().warn(f"[sim/start] {source} received")
        _sim_control_queue.put("start")

    def _queue_stop(self, source: str):
        self.get_logger().warn(f"[sim/stop] {source} received")
        _sim_control_queue.put("stop")

    def _on_start_service(self, _request, response):
        self._queue_start("service")
        response.success = True
        response.message = "Start request queued."
        return response

    def _on_stop_service(self, _request, response):
        self._queue_stop("service")
        response.success = True
        response.message = "Stop request queued."
        return response

    def _on_start_topic(self, _msg: Empty):
        self._queue_start("topic")

    def _on_stop_topic(self, _msg: Empty):
        self._queue_stop("topic")


# ──────────────────────────────────────────────────────────────────────────────
# ROS init
# ──────────────────────────────────────────────────────────────────────────────
rclpy.init()
_ros_node = SimControlNode()
_ros_executor = SingleThreadedExecutor()
_ros_executor.add_node(_ros_node)


def _ros_spin():
    """
    Spin ROS callbacks in a dedicated thread so STOP can be received even if
    simulation_app.update() is temporarily expensive.
    """
    try:
        _ros_executor.spin()
    except Exception as e:
        print(f"[run_isaac_sim][WARN] ROS spin thread exited with error: {e}")


_ros_spin_thread = threading.Thread(target=_ros_spin, name="ros_spin", daemon=True)
_ros_spin_thread.start()

print("[run_isaac_sim] ROS 2 control node ready.")
print("[run_isaac_sim] Simulator is PAUSED. Call /micropolis/sim/start service to begin.")
print("[run_isaac_sim] Entering main loop. Close the window or Ctrl+C to exit.")


# Make sure initial state is really paused
_set_desired_running(False)
_apply_sim_state_if_needed()


# ──────────────────────────────────────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────────────────────────────────────
try:
    while simulation_app.is_running():
        if delegate is not None:
            try:
                delegate.process_pending()
            except Exception as e:
                print(f"[run_isaac_sim][WARN] delegate.process_pending failed: {e}")

        _drain_sim_requests()
        _apply_sim_state_if_needed()

        simulation_app.update()

except KeyboardInterrupt:
    print("[run_isaac_sim] KeyboardInterrupt received, shutting down.")

finally:
    # Force pause before exit
    try:
        _tl.stop()
    except Exception:
        pass

    if delegate is not None:
        try:
            delegate.shutdown()
        except Exception as e:
            print(f"[run_isaac_sim][WARN] delegate.shutdown failed: {e}")

    try:
        _ros_executor.remove_node(_ros_node)
    except Exception:
        pass

    try:
        _ros_executor.shutdown()
    except Exception:
        pass

    try:
        _ros_node.destroy_node()
    except Exception:
        pass

    try:
        rclpy.shutdown()
    except Exception:
        pass

    try:
        if _ros_spin_thread.is_alive():
            _ros_spin_thread.join(timeout=1.0)
    except Exception:
        pass

    simulation_app.close()
    print("[run_isaac_sim] Closed cleanly.")