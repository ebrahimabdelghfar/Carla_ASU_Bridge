#include "carla_telemetry/sensors/gps.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/GeoLocation.h>
#include <carla/geom/Transform.h>

#include <cmath>
#include <tuple>

#include "carla_telemetry/perf_monitor.hpp"

namespace carla_telemetry {

namespace {

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_B = 6356752.31424518;
constexpr double WGS84_E2 = 0.00669437999014;
constexpr double WGS84_E_PRIME2 = 0.00673949674228;

// Geodetic to ECEF
void geodetic_to_ecef(double lat_rad, double lon_rad, double alt, double& x,
                      double& y, double& z) {
  double sin_lat = std::sin(lat_rad);
  double cos_lat = std::cos(lat_rad);
  double sin_lon = std::sin(lon_rad);
  double cos_lon = std::cos(lon_rad);

  double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
  x = (n + alt) * cos_lat * cos_lon;
  y = (n + alt) * cos_lat * sin_lon;
  z = (n * (1.0 - WGS84_E2) + alt) * sin_lat;
}

// ECEF to Geodetic (Bowring's method)
void ecef_to_geodetic(double x, double y, double z, double& lat_rad,
                      double& lon_rad, double& alt) {
  double p = std::sqrt(x * x + y * y);
  if (p < 1e-10) {
    lat_rad = (z > 0.0) ? (M_PI / 2.0) : (-M_PI / 2.0);
    lon_rad = 0.0;
    alt = std::abs(z) - WGS84_B;
    return;
  }

  double theta = std::atan2(z * WGS84_A, p * WGS84_B);
  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  lat_rad = std::atan2(
      z + WGS84_E_PRIME2 * WGS84_B * sin_theta * sin_theta * sin_theta,
      p - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta);
  lon_rad = std::atan2(y, x);

  double sin_lat = std::sin(lat_rad);
  double cos_lat = std::cos(lat_rad);
  alt = p * cos_lat + z * sin_lat -
        WGS84_A * std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
}

// ECEF to ENU
void ecef_to_enu(double x, double y, double z, double origin_lat_rad,
                 double origin_lon_rad, double origin_alt, double enu[3]) {
  double x0, y0, z0;
  geodetic_to_ecef(origin_lat_rad, origin_lon_rad, origin_alt, x0, y0, z0);

  double dx = x - x0;
  double dy = y - y0;
  double dz = z - z0;

  double sin_lat = std::sin(origin_lat_rad);
  double cos_lat = std::cos(origin_lat_rad);
  double sin_lon = std::sin(origin_lon_rad);
  double cos_lon = std::cos(origin_lon_rad);

  enu[0] = -sin_lon * dx + cos_lon * dy;
  enu[1] = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu[2] = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
}

// ENU to ECEF
void enu_to_ecef(const double enu[3], double origin_lat_rad,
                 double origin_lon_rad, double origin_alt, double& x, double& y,
                 double& z) {
  double x0, y0, z0;
  geodetic_to_ecef(origin_lat_rad, origin_lon_rad, origin_alt, x0, y0, z0);

  double sin_lat = std::sin(origin_lat_rad);
  double cos_lat = std::cos(origin_lat_rad);
  double sin_lon = std::sin(origin_lon_rad);
  double cos_lon = std::cos(origin_lon_rad);

  double dx = -sin_lon * enu[0] - sin_lat * cos_lon * enu[1] +
              cos_lat * cos_lon * enu[2];
  double dy = cos_lon * enu[0] - sin_lat * sin_lon * enu[1] +
              cos_lat * sin_lon * enu[2];
  double dz = cos_lat * enu[1] + sin_lat * enu[2];

  x = x0 + dx;
  y = y0 + dy;
  z = z0 + dz;
}

// Convert local ENU offset [x_east, y_north, z_up] (metres) to (lat_rad,
// lon_rad, alt).
std::tuple<double, double, double> reprojection(const double enu[3],
                                                double origin_lat_rad,
                                                double origin_lon_rad,
                                                double origin_alt) {
  double x, y, z;
  enu_to_ecef(enu, origin_lat_rad, origin_lon_rad, origin_alt, x, y, z);
  double lat, lon, alt;
  ecef_to_geodetic(x, y, z, lat, lon, alt);
  return {lat, lon, alt};
}

// WGS-84 (lat, lon, alt) → ENU metres from origin.
void latlon_to_enu_offset(double lat_rad, double lon_rad, double alt,
                          double origin_lat_rad, double origin_lon_rad,
                          double origin_alt, double out[3]) {
  double x, y, z;
  geodetic_to_ecef(lat_rad, lon_rad, alt, x, y, z);
  ecef_to_enu(x, y, z, origin_lat_rad, origin_lon_rad, origin_alt, out);
}

}  // namespace

CarlaGPS::CarlaGPS(carla::client::World& world,
                   carla::SharedPtr<carla::client::Actor> vehicle,
                   const Config& cfg, double origin_lat, double origin_lon,
                   double origin_alt)
    : origin_lat_(origin_lat),
      origin_lon_(origin_lon),
      origin_alt_(origin_alt),
      map_ref_lat_(0.0),
      map_ref_lon_(0.0),
      map_ref_alt_(0.0),
      cfg_(cfg),
      rng_(std::random_device{}()) {
  // Query CARLA map's built-in geo-reference (from OpenDRIVE header).
  // CARLA's GNSS sensor outputs lat/lon relative to THIS origin,
  // so we need it to correctly reproject to user's global_coordinates.
  auto map = world.GetMap();
  if (map) {
    auto geo_ref = map->GetGeoReference();
    map_ref_lat_ = geo_ref.latitude;
    map_ref_lon_ = geo_ref.longitude;
    map_ref_alt_ = geo_ref.altitude;
  }

  // Initialize state
  state_.latitude = origin_lat;
  state_.longitude = origin_lon;
  state_.altitude = origin_alt;
  state_.latitude_gt = origin_lat;
  state_.longitude_gt = origin_lon;
  state_.altitude_gt = origin_alt;
  state_.fix_type = 3;
  state_.eph = cfg.eph;
  state_.epv = cfg.epv;
  state_.satellites = cfg.satellites_visible;

  // Spawn CARLA GNSS sensor
  auto bp_lib = world.GetBlueprintLibrary();
  auto bp = *bp_lib->Find("sensor.other.gnss");

  carla::geom::Transform transform(
      carla::geom::Location(cfg.sp_x, cfg.sp_y, cfg.sp_z),
      carla::geom::Rotation(cfg.sp_pitch, cfg.sp_yaw, cfg.sp_roll));

  auto actor = world.SpawnActor(bp, transform, vehicle.get());
  sensor_ = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
  sensor_->Listen([this](auto data) { this->on_gnss(data); });
}

void CarlaGPS::on_gnss(carla::SharedPtr<carla::sensor::SensorData> data) {
  auto gnss =
      boost::dynamic_pointer_cast<carla::sensor::data::GnssMeasurement>(data);
  if (!gnss) return;
  // Rate the server delivers GNSS measurements to this client (inter-arrival).
  perf.record_interval("gps.server_dt");
  std::lock_guard<std::mutex> lk(lock_);
  last_gnss_ = gnss;
}

std::optional<GpsState> CarlaGPS::update(double vx, double vy, double vz,
                                         double dt) {
  carla::SharedPtr<carla::sensor::data::GnssMeasurement> gnss;
  {
    std::lock_guard<std::mutex> lk(lock_);
    gnss = last_gnss_;
  }
  if (!gnss) return std::nullopt;

  // CARLA GNSS outputs absolute lat/lon using the map's built-in geo-reference.
  // We need to: (1) convert GNSS → ENU relative to map's geo-ref origin,
  //             (2) then reproject those ENU offsets from user's
  //             global_coordinates.
  double lat_gnss_deg = gnss->GetLatitude();
  double lon_gnss_deg = gnss->GetLongitude();
  double alt_gnss = gnss->GetAltitude();

  double map_ref_lat_rad = map_ref_lat_ * DEG_TO_RAD;
  double map_ref_lon_rad = map_ref_lon_ * DEG_TO_RAD;
  double origin_lat_rad = origin_lat_ * DEG_TO_RAD;
  double origin_lon_rad = origin_lon_ * DEG_TO_RAD;

  // Step 1: Convert GNSS (map-referenced) → ENU metres relative to map origin
  double enu_gt[3];
  latlon_to_enu_offset(lat_gnss_deg * DEG_TO_RAD, lon_gnss_deg * DEG_TO_RAD,
                       alt_gnss, map_ref_lat_rad, map_ref_lon_rad, map_ref_alt_,
                       enu_gt);

  // Ground-truth: reproject ENU from user's global_coordinates origin
  auto [lat_gt_rad, lon_gt_rad, alt_gt] =
      reprojection(enu_gt, origin_lat_rad, origin_lon_rad, origin_alt_);
  double lat_gt_deg = lat_gt_rad * RAD_TO_DEG;
  double lon_gt_deg = lon_gt_rad * RAD_TO_DEG;

  // Noise model (identical to Micropolis)
  double sqrt_dt = std::sqrt(dt);

  // Random-walk bias step
  double rw[3] = {
      cfg_.gps_xy_random_walk * sqrt_dt * normal_(rng_),
      cfg_.gps_xy_random_walk * sqrt_dt * normal_(rng_),
      cfg_.gps_z_random_walk * sqrt_dt * normal_(rng_),
  };

  // Position white noise (using discrete standard deviation, so no sqrt_dt
  // scaling)
  double noise_pos[3] = {
      cfg_.gps_xy_noise_density * normal_(rng_),
      cfg_.gps_xy_noise_density * normal_(rng_),
      cfg_.gps_z_noise_density * normal_(rng_),
  };

  // Gauss-Markov bias integration: x_{k+1} = x_k + rw - x_k * (dt / tau)
  for (int i = 0; i < 3; ++i) {
    gps_bias_[i] += rw[i] - gps_bias_[i] * dt / cfg_.gps_correlation_time;
  }

  // Noisy ENU position
  double enu_noisy[3] = {
      enu_gt[0] + noise_pos[0] + gps_bias_[0],
      enu_gt[1] + noise_pos[1] + gps_bias_[1],
      enu_gt[2] + noise_pos[2] + gps_bias_[2],
  };

  // Cache both ENU vectors so get_latest_enu() can serve them without
  // recomputation, together with the identity of the fix they came from so
  // enu_source() can report it. Written under lock_ because the odometry loop
  // reads the pair (position, source frame) from another thread and must never
  // see one fix's frame number next to another fix's position.
  {
    std::lock_guard<std::mutex> lk(lock_);
    enu_gt_[0] = enu_gt[0];
    enu_gt_[1] = enu_gt[1];
    enu_gt_[2] = enu_gt[2];
    enu_noisy_[0] = enu_noisy[0];
    enu_noisy_[1] = enu_noisy[1];
    enu_noisy_[2] = enu_noisy[2];
    enu_frame_ = static_cast<uint64_t>(gnss->GetFrame());
    enu_sim_time_ = gnss->GetTimestamp();
    enu_valid_ = true;
  }

  // Step 2: Reproject noisy ENU from user's global_coordinates origin
  auto [lat_noisy_rad, lon_noisy_rad, alt_noisy] =
      reprojection(enu_noisy, origin_lat_rad, origin_lon_rad, origin_alt_);

  // Velocity (CARLA frame: X=East, Y=South, Z=up)
  // Therefore: North = -South = -vy, East = vx, Down = -vz
  double vxy_noise = cfg_.gps_vxy_noise_density;  // discrete stddev, no sqrt_dt
  double vz_noise = cfg_.gps_vz_noise_density;
  double vel_north = -vy + vxy_noise * normal_(rng_);
  double vel_east = vx + vxy_noise * normal_(rng_);
  double vel_down = -vz + vz_noise * normal_(rng_);
  double speed = std::sqrt(vx * vx + vy * vy);

  state_.latitude = lat_noisy_rad * RAD_TO_DEG;
  state_.longitude = lon_noisy_rad * RAD_TO_DEG;
  state_.altitude = alt_noisy;
  state_.latitude_gt = lat_gt_deg;
  state_.longitude_gt = lon_gt_deg;
  state_.altitude_gt = alt_gt;
  state_.speed = speed;
  state_.velocity_north = vel_north;
  state_.velocity_east = vel_east;
  state_.velocity_down = vel_down;
  state_.fix_type = 3;
  state_.eph = cfg_.eph;
  state_.epv = cfg_.epv;
  state_.satellites = cfg_.satellites_visible;

  return state_;
}

void CarlaGPS::destroy() {
  if (sensor_) {
    try {
      sensor_->Stop();
      sensor_->Destroy();
    } catch (...) {
    }
    sensor_ = nullptr;
  }
}

bool CarlaGPS::get_latest_gt(double& lat, double& lon, double& alt) {
  std::lock_guard<std::mutex> lk(lock_);
  if (!last_gnss_) return false;

  // Convert GNSS (map-referenced) → ENU → reproject from user's origin
  double map_ref_lat_rad = map_ref_lat_ * DEG_TO_RAD;
  double map_ref_lon_rad = map_ref_lon_ * DEG_TO_RAD;
  double origin_lat_rad = origin_lat_ * DEG_TO_RAD;
  double origin_lon_rad = origin_lon_ * DEG_TO_RAD;

  double enu[3];
  latlon_to_enu_offset(last_gnss_->GetLatitude() * DEG_TO_RAD,
                       last_gnss_->GetLongitude() * DEG_TO_RAD,
                       last_gnss_->GetAltitude(), map_ref_lat_rad,
                       map_ref_lon_rad, map_ref_alt_, enu);

  auto [lat_rad, lon_rad, alt_gt] =
      reprojection(enu, origin_lat_rad, origin_lon_rad, origin_alt_);
  lat = lat_rad * RAD_TO_DEG;
  lon = lon_rad * RAD_TO_DEG;
  alt = alt_gt;
  return true;
}

bool CarlaGPS::get_latest_enu(double& east, double& north, double& up,
                              bool use_noise) const {
  std::lock_guard<std::mutex> lk(lock_);
  if (!enu_valid_) return false;
  const double* src = use_noise ? enu_noisy_ : enu_gt_;
  east = src[0];
  north = src[1];
  up = src[2];
  return true;
}

bool CarlaGPS::enu_source(uint64_t& frame, double& sim_time) const {
  std::lock_guard<std::mutex> lk(lock_);
  if (!enu_valid_) return false;
  frame = enu_frame_;
  sim_time = enu_sim_time_;
  return true;
}

}  // namespace carla_telemetry
