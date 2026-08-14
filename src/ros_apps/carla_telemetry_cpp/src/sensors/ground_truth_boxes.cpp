#include "carla_telemetry/sensors/ground_truth_boxes.hpp"

#include <carla/client/ActorList.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector3D.h>
#include <carla/rpc/ObjectLabel.h>

#include <cmath>

#include "carla_telemetry/sensors/odometry.hpp"  // euler_to_quaternion

namespace carla_telemetry {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kMovingSpeed = 0.5;  // m/s threshold for "moving"

// Classify a CARLA actor by its blueprint type_id into a Waymo class.
// Returns empty string for actors that should be skipped.
std::string classify(const carla::SharedPtr<carla::client::Actor>& a) {
  const std::string& type_id = a->GetTypeId();
  if (type_id.rfind("vehicle.", 0) == 0) {
    // Cyclist = two-wheeled vehicle blueprint.
    for (const auto& attr : a->GetAttributes()) {
      if (attr.GetId() == "number_of_wheels") {
        return attr.GetValue() == "2" ? "cyclist" : "vehicle";
      }
    }
    return "vehicle";
  }
  if (type_id.rfind("walker.", 0) == 0) return "pedestrian";
  // Signs only — traffic lights are not sign objects.
  if (type_id.rfind("traffic.", 0) == 0 &&
      type_id.find("traffic_light") == std::string::npos) {
    return "sign";
  }
  return "";
}
}  // namespace

CarlaGroundTruthBoxes::CarlaGroundTruthBoxes(const std::string& frame_id,
                                             const RangeWindow& window)
    : frame_id_(frame_id), window_(window) {}

GroundTruthBoxes CarlaGroundTruthBoxes::get_boxes(
    carla::client::World& world, carla::SharedPtr<carla::client::Actor> ego) {
  GroundTruthBoxes out;
  out.frame_id = frame_id_;
  if (!ego) return out;

  const auto ego_id = ego->GetId();
  const auto ego_tf = ego->GetTransform();
  const auto& er = ego_tf.rotation;
  // Ego yaw in ROS convention (CARLA yaw is CW/left-handed → negate).
  const double ego_yaw_ros = -static_cast<double>(er.yaw) * kDeg2Rad;
  const double cos_e = std::cos(ego_yaw_ros);
  const double sin_e = std::sin(ego_yaw_ros);

  // Is an ego-relative center (ROS base_link frame) inside the detection
  // window?
  const double half_fov = window_.horizontal_fov * 0.5 * kDeg2Rad;
  const bool fov_all = window_.horizontal_fov >= 360.0;
  auto in_window = [&](double px, double py, double pz) -> bool {
    const double r = std::sqrt(px * px + py * py);
    if (window_.max_range > 0.0 && r > window_.max_range) return false;
    if (window_.min_range > 0.0 && r < window_.min_range) return false;
    if (!fov_all && std::abs(std::atan2(py, px)) > half_fov) return false;
    if (window_.z_max > window_.z_min &&
        (pz < window_.z_min || pz > window_.z_max))
      return false;
    return true;
  };

  // Build a box from a WORLD-space center + rotation (CARLA frame), expressed
  // relative to the ego base_link. velocity is CARLA world-frame (0 for
  // static).
  auto make_box = [&](const std::string& label, uint32_t track_id, bool is_ego,
                      carla::geom::Vector3D center,  // CARLA world coords
                      float w_roll, float w_pitch, float w_yaw,
                      const carla::geom::Vector3D& extent, float vel_x,
                      float vel_y) {
    GroundTruthBox b;
    b.label = label;
    b.track_id = track_id;
    b.score = 1.0f;
    b.is_ego = is_ego;

    // World → ego base_link, then CARLA(LH) → ROS(RH) Y flip.
    ego_tf.InverseTransformPoint(center);
    b.px = static_cast<double>(center.x);
    b.py = -static_cast<double>(center.y);
    b.pz = static_cast<double>(center.z);

    // Orientation relative to ego, CARLA→ROS (negate pitch & yaw).
    double roll = static_cast<double>(w_roll - er.roll) * kDeg2Rad;
    double pitch = -static_cast<double>(w_pitch - er.pitch) * kDeg2Rad;
    double yaw = -static_cast<double>(w_yaw - er.yaw) * kDeg2Rad;
    euler_to_quaternion(roll, pitch, yaw, b.qx, b.qy, b.qz, b.qw);

    // Size: extent is half-dimensions → full box scale.
    b.sx = 2.0 * static_cast<double>(extent.x);
    b.sy = 2.0 * static_cast<double>(extent.y);
    b.sz = 2.0 * static_cast<double>(extent.z);

    // Velocity: CARLA world → ROS world (Y flip) → ego base_link frame.
    const double vwx = static_cast<double>(vel_x);
    const double vwy = -static_cast<double>(vel_y);
    b.vx = vwx * cos_e + vwy * sin_e;
    b.vy = -vwx * sin_e + vwy * cos_e;

    const double speed = std::sqrt(static_cast<double>(vel_x) * vel_x +
                                   static_cast<double>(vel_y) * vel_y);
    b.attribute = speed > kMovingSpeed ? "moving" : "stopped";

    if (!std::isfinite(b.px) || !std::isfinite(b.py) || !std::isfinite(b.pz) ||
        !std::isfinite(b.qx) || !std::isfinite(b.qy) || !std::isfinite(b.qz) ||
        !std::isfinite(b.qw) || !std::isfinite(b.sx) || !std::isfinite(b.sy) ||
        !std::isfinite(b.sz) || !std::isfinite(b.vx) || !std::isfinite(b.vy)) {
      return;
    }

    // Drop ghost boxes that have zero or near-zero scale
    if (b.sx < 1e-3 || b.sy < 1e-3 || b.sz < 1e-3) {
      return;
    }

    // LiDAR-style detection window. Ego always kept; others must fall inside
    // the configured radial range / horizontal FOV / vertical band.
    if (!is_ego && !in_window(b.px, b.py, b.pz)) return;

    out.boxes.push_back(std::move(b));
  };

  // Dynamic actors (vehicles / cyclists / pedestrians / signs)
  auto actors = world.GetActors();
  for (auto a : *actors) {
    std::string label = classify(a);
    if (label.empty()) continue;

    // Bounding box is static per actor but the fork's GetBoundingBox()
    // does a blocking per-actor RPC — fetch once and cache by id.
    const uint32_t id = static_cast<uint32_t>(a->GetId());
    auto cache_it = bbox_cache_.find(id);
    if (cache_it == bbox_cache_.end()) {
      cache_it = bbox_cache_.emplace(id, a->GetBoundingBox()).first;
    }
    const auto& bbox = cache_it->second;  // location/rotation are actor-local
    const auto a_tf = a->GetTransform();

    // Compose actor-local bbox center into world coords.
    carla::geom::Vector3D center(bbox.location.x, bbox.location.y,
                                 bbox.location.z);
    a_tf.TransformPoint(center);

    const auto v = a->GetVelocity();
    make_box(label, id, id == ego_id, center, a_tf.rotation.roll,
             a_tf.rotation.pitch, a_tf.rotation.yaw + bbox.rotation.yaw,
             bbox.extent, v.x, v.y);
  }

  // Parked / map-baked vehicles (environment objects)
  // Not actors, so GetActors() misses them. They never move → fetch once.
  if (!env_vehicles_loaded_) {
    env_vehicles_ = world.GetEnvironmentObjects(
        static_cast<uint8_t>(carla::rpc::CityObjectLabel::Car));
    env_vehicles_loaded_ = true;
  }
  for (const auto& eo : env_vehicles_) {
    const auto& bb = eo.bounding_box;  // world-space for environment objects
    carla::geom::Vector3D center(bb.location.x, bb.location.y, bb.location.z);
    make_box("vehicle", static_cast<uint32_t>(eo.id), false, center,
             bb.rotation.roll, bb.rotation.pitch, bb.rotation.yaw, bb.extent,
             0.0f, 0.0f);
  }

  return out;
}

}  // namespace carla_telemetry
