#pragma once

#include <carla/client/Actor.h>
#include <carla/client/World.h>
#include <carla/geom/Location.h>

#include <memory>
#include <string>
#include <vector>

namespace carla_telemetry {

class CarlaDynamicProps {
 public:
  struct Config {
    bool enabled = false;
    bool spawn_on_roads = true;
    int count = 0;
    float max_distance = 10.0f;
    float min_distance_from_ego = 8.0f;
    float prop_to_prop_distance = 2.5f;
    float spawn_height = 1.0f;
    std::vector<std::string> blueprints;
  };

  // world must outlive this object.
  CarlaDynamicProps(carla::client::World& world, const Config& cfg,
                    const carla::geom::Location& ego_loc);

  ~CarlaDynamicProps();

  void destroy();

 private:
  std::vector<carla::SharedPtr<carla::client::Actor>> props_;
};

}  // namespace carla_telemetry
