"""
| File: gps.py
| Description: CARLA GPS sensor with realistic noise model (Gauss-Markov + white noise).
               Wraps the CARLA sensor.other.gnss sensor and applies the same
               noise model as Micropolis.Telemetry.sensors.gps.GPS.

               The Gauss-Markov bias drift and white-noise are applied additively
               in metres (ENU), then converted back to lat/lon degrees via a
               spherical-Earth reprojection identical to the Micropolis model.
"""
__all__ = ["CarlaGPS"]

import threading
import time
import weakref

import numpy as np

# ---------------------------------------------------------------------------
# Minimal WGS-84 reprojection helper (identical to Micropolis GPS)
# ---------------------------------------------------------------------------
EARTH_RADIUS = 6_353_000.0  # metres


def _reprojection(position: np.ndarray, origin_lat_rad: float, origin_lon_rad: float):
    """Convert a local ENU offset [x_east, y_north, z_up] (metres) to (lat_rad, lon_rad)."""
    x_rad = position[1] / EARTH_RADIUS   # north component
    y_rad = position[0] / EARTH_RADIUS   # east component
    c = np.sqrt(x_rad * x_rad + y_rad * y_rad)
    sin_c = np.sin(c)
    cos_c = np.cos(c)

    if c > 1e-10:
        lat = np.arcsin(
            cos_c * np.sin(origin_lat_rad)
            + (x_rad * sin_c * np.cos(origin_lat_rad)) / c
        )
        lon = origin_lon_rad + np.arctan2(
            y_rad * sin_c,
            c * np.cos(origin_lat_rad) * cos_c
            - x_rad * np.sin(origin_lat_rad) * sin_c,
        )
    else:
        lat = origin_lat_rad
        lon = origin_lon_rad

    return lat, lon


def _latlon_to_enu_offset(lat_rad: float, lon_rad: float,
                           origin_lat_rad: float, origin_lon_rad: float) -> np.ndarray:
    """Approximate inverse: WGS-84 (lat, lon) → ENU metres from origin."""
    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad
    y_north = dlat * EARTH_RADIUS                                        # North (ENU Y)
    x_east = dlon * EARTH_RADIUS * np.cos(origin_lat_rad)               # East  (ENU X)
    return np.array([x_east, y_north, 0.0])


# ---------------------------------------------------------------------------
# CARLA GPS wrapper
# ---------------------------------------------------------------------------

class CarlaGPS:
    """CARLA GNSS sensor wrapper with Gauss-Markov + white-noise model.

    The noise model is identical to ``Micropolis.Telemetry.sensors.gps.GPS``:
    - A Gauss-Markov random-walk bias is integrated each update step.
    - White noise is drawn independently each step.
    - Noise is added in ENU metres, then reprojected to WGS-84.

    Output state dict keys:
    =================== =============================================
    Key                 Value
    =================== =============================================
    ``latitude``        Noisy latitude in **degrees**
    ``longitude``       Noisy longitude in **degrees**
    ``altitude``        Noisy altitude in **metres** (ASL)
    ``latitude_gt``     Ground-truth latitude (degrees)
    ``longitude_gt``    Ground-truth longitude (degrees)
    ``altitude_gt``     Ground-truth altitude (metres)
    ``speed``           Horizontal speed (m/s)
    ``velocity_north``  North velocity (m/s)
    ``velocity_east``   East velocity (m/s)
    ``velocity_down``   Down velocity (m/s)
    ``fix_type``        3 (fixed GPS fix)
    ``eph``             Horizontal position error estimate (m)
    ``epv``             Vertical   position error estimate (m)
    ``satellites``      Simulated visible satellites count
    =================== =============================================
    """

    def __init__(self, carla_world, vehicle_actor, config: dict,
                 origin_lat: float, origin_lon: float, origin_alt: float):
        """
        Args:
            carla_world:    carla.World instance.
            vehicle_actor:  The ego vehicle actor to attach GNSS to.
            config:         GPS config dict from carla_interface_config.yaml.
            origin_lat/lon/alt: World-origin geographic coordinates.
        """
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._origin_alt = origin_alt
        self._update_rate = float(config.get("update_rate", 10.0))

        # Noise model parameters (identical to Micropolis)
        self._gps_bias = np.zeros(3)
        self._gps_xy_random_walk = float(config.get("gps_xy_random_walk", 2.0))
        self._gps_z_random_walk = float(config.get("gps_z_random_walk", 4.0))
        self._gps_correlation_time = float(config.get("gps_correlation_time", 60.0))

        self._gps_xy_noise_density = float(config.get("gps_xy_noise_density", 2e-4))
        self._gps_z_noise_density = float(config.get("gps_z_noise_density", 4e-4))
        self._gps_vxy_noise_density = float(config.get("gps_vxy_noise_density", 0.2))
        self._gps_vz_noise_density = float(config.get("gps_vz_noise_density", 0.4))

        self._eph = float(config.get("eph", 1.0))
        self._epv = float(config.get("epv", 1.0))
        self._satellites = int(config.get("satellites_visible", 10))

        self._lock = threading.Lock()
        self._last_gnss = None        # raw carla.GnssMeasurement
        self._last_velocity = None    # carla.Vector3D

        # Current published state
        self._state = {
            "latitude": origin_lat, "longitude": origin_lon, "altitude": origin_alt,
            "latitude_gt": origin_lat, "longitude_gt": origin_lon, "altitude_gt": origin_alt,
            "speed": 0.0, "velocity_north": 0.0, "velocity_east": 0.0, "velocity_down": 0.0,
            "fix_type": 3, "eph": self._eph, "epv": self._epv, "satellites": self._satellites,
        }

        # Spawn the CARLA GNSS sensor
        sp_cfg = config.get("spawn_point", {})
        import carla
        bp = carla_world.get_blueprint_library().find("sensor.other.gnss")

        # Apply CARLA noise attributes if configured
        for attr in ("noise_alt_stddev", "noise_lat_stddev", "noise_lon_stddev",
                     "noise_alt_bias", "noise_lat_bias", "noise_lon_bias"):
            val = config.get(attr)
            if val is not None and bp.has_attribute(attr):
                bp.set_attribute(attr, str(float(val)))

        transform = carla.Transform(
            carla.Location(
                x=float(sp_cfg.get("x", 1.0)),
                y=float(sp_cfg.get("y", 0.0)),
                z=float(sp_cfg.get("z", 2.8)),
            ),
            carla.Rotation(
                roll=float(sp_cfg.get("roll", 0.0)),
                pitch=float(sp_cfg.get("pitch", 0.0)),
                yaw=float(sp_cfg.get("yaw", 0.0)),
            ),
        )
        self._sensor = carla_world.spawn_actor(bp, transform, attach_to=vehicle_actor)

        # Weak-ref callback to avoid circular reference
        weak_self = weakref.ref(self)
        self._sensor.listen(lambda data: CarlaGPS._on_gnss(weak_self, data))

    # ------------------------------------------------------------------
    # CARLA callback (fires on CARLA worker thread)
    # ------------------------------------------------------------------

    @staticmethod
    def _on_gnss(weak_self, data):
        """Store latest raw GNSS measurement (thread-safe)."""
        self = weak_self()
        if self is None:
            return
        with self._lock:
            self._last_gnss = data

    # ------------------------------------------------------------------
    # Public API (called from the ROS 2 timer on the main thread)
    # ------------------------------------------------------------------

    def update(self, vehicle_velocity, dt: float):
        """Compute noisy GPS reading and return state dict.

        Args:
            vehicle_velocity: carla.Vector3D — current vehicle velocity (m/s, CARLA frame).
            dt:               Time elapsed since last call (seconds).

        Returns:
            State dict or ``None`` if rate limit not yet reached.
        """
        with self._lock:
            gnss = self._last_gnss

        if gnss is None:
            return None

        # ── Ground-truth lat/lon/alt from CARLA GNSS ──────────────────
        lat_gt_deg = gnss.latitude
        lon_gt_deg = gnss.longitude
        alt_gt = gnss.altitude + self._origin_alt

        origin_lat_rad = np.radians(self._origin_lat)
        origin_lon_rad = np.radians(self._origin_lon)

        # Convert GT to ENU metres for noise injection
        enu_gt = _latlon_to_enu_offset(
            np.radians(lat_gt_deg), np.radians(lon_gt_deg),
            origin_lat_rad, origin_lon_rad,
        )
        enu_gt[2] = gnss.altitude  # use CARLA altitude directly

        # ── Noise model (identical to Micropolis) ─────────────────────
        # Random-walk bias step
        rw = np.array([
            self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn(),
            self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn(),
            self._gps_z_random_walk  * np.sqrt(dt) * np.random.randn(),
        ])

        # Position white noise
        noise_pos = np.array([
            self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn(),
            self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn(),
            self._gps_z_noise_density  * np.sqrt(dt) * np.random.randn(),
        ])

        # Gauss-Markov bias integration
        self._gps_bias += rw * dt - self._gps_bias / self._gps_correlation_time

        # Noisy ENU position
        enu_noisy = enu_gt + noise_pos + self._gps_bias

        # Reproject to lat/lon
        lat_noisy_rad, lon_noisy_rad = _reprojection(enu_noisy, origin_lat_rad, origin_lon_rad)

        # ── Velocity (CARLA frame: X=forward, Y=right, Z=up → NED) ────
        # CARLA velocity: X=forward(North), Y=right(East), Z=up
        vx = float(vehicle_velocity.x)   # forward ≈ North
        vy = float(vehicle_velocity.y)   # right   ≈ East
        vz = float(vehicle_velocity.z)   # up

        # Add velocity noise
        vxy_noise = self._gps_vxy_noise_density * np.sqrt(dt)
        vz_noise = self._gps_vz_noise_density  * np.sqrt(dt)
        vel_north = vx + vxy_noise * np.random.randn()
        vel_east  = vy + vxy_noise * np.random.randn()
        vel_down  = -vz + vz_noise * np.random.randn()

        speed = float(np.sqrt(vx**2 + vy**2))

        self._state = {
            "latitude":       float(np.degrees(lat_noisy_rad)),
            "longitude":      float(np.degrees(lon_noisy_rad)),
            "altitude":       float(enu_noisy[2] + self._origin_alt),
            "latitude_gt":    lat_gt_deg,
            "longitude_gt":   lon_gt_deg,
            "altitude_gt":    alt_gt,
            "speed":          speed,
            "velocity_north": vel_north,
            "velocity_east":  vel_east,
            "velocity_down":  vel_down,
            "fix_type":       3,
            "eph":            self._eph,
            "epv":            self._epv,
            "satellites":     self._satellites,
        }
        return self._state

    @property
    def state(self):
        return self._state

    def destroy(self):
        """Stop the CARLA GNSS sensor."""
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
            except Exception:
                pass
            self._sensor = None
