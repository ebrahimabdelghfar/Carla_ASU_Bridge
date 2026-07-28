"""
| File: battery.py
| Description: Direct port of Micropolis.Telemetry.sensors.battery.Battery.
               Software battery simulation model — no CARLA sensor required.
               Driven by the ego vehicle speed obtained from vehicle.get_velocity().

Physics reference:
  https://gazebosim.org/api/sim/7/battery.html
  gz-sim/src/systems/linear_battery_plugin/LinearBatteryPlugin.cc

Two consumption modes (selectable in carla_interface_config.yaml):
  "constant"       - fixed 'power_load' watts regardless of motion
  "velocity_based" - baseline + extra power proportional to vehicle speed
"""
__all__ = ["CarlaBattery"]

import time
import numpy as np


class CarlaBattery:
    """Gazebo-compatible linear battery simulation for CARLA.

    Identical logic to ``Micropolis.Telemetry.sensors.battery.Battery``.

    Output state dict keys:
    ===================== =====================================================
    Key                   Value
    ===================== =====================================================
    ``voltage``           Terminal voltage (V)
    ``open_circuit_v``    Open-circuit voltage (V)
    ``current``           Smoothed draw current (A)
    ``charge``            Remaining charge (Ah)
    ``charge_fraction``   Remaining charge 0 – 1
    ``percentage``        Remaining charge 0 – 100 %
    ``power_load``        Instantaneous total power draw (W)
    ``is_depleted``       True when charge ≤ 0
    ===================== =====================================================
    """

    def __init__(self, config: dict = None):
        if config is None:
            config = {}
        """
        Args:
            config: Battery config dict from carla_interface_config.yaml (all keys optional).
        """
        self._update_rate = float(config.get("update_rate", 10.0))

        # ── Electrical parameters ─────────────────────────────────────
        self._v_nom      = float(config.get("voltage", 12.6))
        self._e0         = float(config.get("open_circuit_voltage_constant_coef", 12.6))
        self._e1         = float(config.get("open_circuit_voltage_linear_coef", -2.8))
        self._capacity   = float(config.get("capacity", 10.0))
        self._charge     = float(config.get("initial_charge", self._capacity))
        self._resistance = float(config.get("resistance", 0.061))
        self._tau        = float(config.get("smooth_current_tau", 2.0))

        # ── Power consumers ───────────────────────────────────────────
        self._base_power_load = float(config.get("power_load", 50.0))
        self._start_draining  = bool(config.get("start_draining", True))
        self._is_draining     = self._start_draining
        self._enable_recharge = bool(config.get("enable_recharge", False))
        self._charging_time   = float(config.get("charging_time", 1.0))   # hours
        self._is_charging     = False

        # ── Consumption mode ──────────────────────────────────────────
        self._consumption_mode = config.get("consumption_mode", "velocity_based")
        self._power_per_speed  = float(config.get("power_per_speed", 15.0))

        # ── Runtime state ─────────────────────────────────────────────
        self._smoothed_current = 0.0
        self._is_depleted      = False
        self._state = self._make_state()

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self):
        return self._state

    @property
    def is_depleted(self) -> bool:
        return self._is_depleted

    @property
    def is_draining(self) -> bool:
        return self._is_draining

    # ------------------------------------------------------------------
    # Charging control (called from ROS 2 service callbacks)
    # ------------------------------------------------------------------

    def start_draining(self):
        """Begin discharging."""
        self._is_depleted = False
        self._is_draining = True
        self._is_charging = False

    def stop_draining(self):
        """Pause discharging."""
        self._is_draining = False

    def start_charging(self):
        """Begin recharging."""
        self._is_charging = True
        self._is_draining = False

    def stop_charging(self):
        """Stop recharging."""
        self._is_charging = False

    # ------------------------------------------------------------------
    # Update (called from ROS 2 timer on main thread)
    # ------------------------------------------------------------------

    def update(self, speed: float, dt: float):
        """Advance the battery model.

        Args:
            speed: Horizontal vehicle speed in m/s.
            dt:    Elapsed wall-clock time since last call (seconds).

        Returns:
            Updated state dict, or ``None`` if rate limit not reached.
        """
        if self._is_depleted and not self._is_charging:
            return self._state

        # ── Charging path ─────────────────────────────────────────────
        if self._is_charging:
            charge_rate = self._capacity / max(self._charging_time, 0.001)  # Ah/h
            self._charge = min(
                self._capacity,
                self._charge + charge_rate * (dt / 3600.0),
            )
            if self._charge >= self._capacity:
                self._is_charging = False
                self._is_depleted = False
            self._state = self._make_state()
            return self._state

        # ── Discharging path ──────────────────────────────────────────
        if not self._is_draining:
            return self._state

        if self._consumption_mode == "velocity_based":
            total_power = self._base_power_load + self._power_per_speed * speed
        else:  # "constant"
            total_power = self._base_power_load

        v_oc = self._e0 + self._e1 * (1.0 - self._charge / self._capacity)
        v_oc = max(v_oc, 0.01)

        i_instant = total_power / v_oc

        if self._tau > 0.0:
            self._smoothed_current += (dt / self._tau) * (i_instant - self._smoothed_current)
        else:
            self._smoothed_current = i_instant

        self._charge -= self._smoothed_current * (dt / 3600.0)
        self._charge = max(self._charge, 0.0)

        if self._charge <= 0.0:
            self._is_depleted = True
            self._is_draining = False

        self._state = self._make_state(total_power=total_power, v_oc=v_oc)
        return self._state

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _make_state(self, total_power: float = 0.0, v_oc: float = None) -> dict:
        if v_oc is None:
            v_oc = self._e0 + self._e1 * (1.0 - self._charge / self._capacity)

        v_terminal = v_oc - self._smoothed_current * self._resistance
        charge_fraction = max(0.0, min(1.0, self._charge / self._capacity))

        return {
            "voltage":         float(v_terminal),
            "open_circuit_v":  float(v_oc),
            "current":         float(self._smoothed_current),
            "charge":          float(self._charge),
            "charge_fraction": float(charge_fraction),
            "percentage":      float(charge_fraction * 100.0),
            "power_load":      float(total_power),
            "is_depleted":     self._is_depleted,
        }
