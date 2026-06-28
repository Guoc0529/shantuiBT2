#include "center_articulation_planner/core/config_loader.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace
{
  template <typename T>
  T ParseOrDefault(const std::string &json, const std::string &key, T def)
  {
    std::regex number_regex("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)");
    std::smatch m;
    if (std::regex_search(json, m, number_regex) && m.size() > 1)
    {
      std::istringstream iss(m[1].str());
      T v{};
      iss >> v;
      if (!iss.fail())
      {
        return v;
      }
    }
    return def;
  }

  inline bool ParseOrDefault(const std::string &json, const std::string &key, bool def)
  {
    std::regex bool_regex("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (std::regex_search(json, m, bool_regex) && m.size() > 1)
    {
      return m[1].str() == "true";
    }
    return def;
  }

  template <typename T>
  T ParseOrDefault(const std::string &json, const char *key, T def)
  {
    return ParseOrDefault(json, std::string(key), def);
  }
} // namespace

namespace jhzx::center_articulation_planner
{
  bool LoadPlannerConfig(const std::string &path,
                         global_planner::types::VehicleShape &shape_normal,
                         global_planner::types::VehicleShape &shape_unload,
                         global_planner::GlobalPlanner::Config &cfg,
                         std::string &err)
  {
    std::ifstream ifs(path);
    if (!ifs)
    {
      err = "Failed to open config file: " + path;
      return false;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string json = buffer.str();

    // Vehicle shape (normal)
    shape_normal.front_length = ParseOrDefault(json, "front_length", shape_normal.front_length);
    shape_normal.rear_length = ParseOrDefault(json, "rear_length", shape_normal.rear_length);
    shape_normal.width = ParseOrDefault(json, "width", shape_normal.width);
    shape_normal.front_to_hitch = ParseOrDefault(json, "front_to_hitch", shape_normal.front_to_hitch);
    shape_normal.rear_to_hitch = ParseOrDefault(json, "rear_to_hitch", shape_normal.rear_to_hitch);
    shape_normal.tool_length = ParseOrDefault(json, "tool_length", shape_normal.tool_length);
    shape_normal.safety_margin = ParseOrDefault(json, "safety_margin", shape_normal.safety_margin);

    // Unload shape defaults to normal unless caller overwrites after load.
    shape_unload = shape_normal;

    // Planner params
    auto &p = cfg.params;
    p.max_steering_angle_deg = ParseOrDefault(json, "max_steering_angle_deg", p.max_steering_angle_deg);
    p.steering_angle_discrete_num = ParseOrDefault(json, "steering_angle_discrete_num", p.steering_angle_discrete_num);
    p.wheel_base = ParseOrDefault(json, "wheel_base", p.wheel_base);
    p.state_grid_resolution = ParseOrDefault(json, "state_grid_resolution", p.state_grid_resolution);
    p.segment_length = ParseOrDefault(json, "segment_length", p.segment_length);
    p.segment_length_discrete_num = ParseOrDefault(json, "segment_length_discrete_num", p.segment_length_discrete_num);
    p.min_turning_radius = ParseOrDefault(json, "min_turning_radius", p.min_turning_radius);
    p.steering_penalty = ParseOrDefault(json, "steering_penalty", p.steering_penalty);
    p.reversing_penalty = ParseOrDefault(json, "reversing_penalty", p.reversing_penalty);
    p.steering_change_penalty = ParseOrDefault(json, "steering_change_penalty", p.steering_change_penalty);
    p.w_ref = ParseOrDefault(json, "w_ref", p.w_ref);
    p.shot_distance = ParseOrDefault(json, "shot_distance", p.shot_distance);
    p.grid_size_phi = ParseOrDefault(json, "grid_size_phi", p.grid_size_phi);
    p.goal_pos_tol = ParseOrDefault(json, "goal_pos_tol", p.goal_pos_tol);
    p.goal_yaw_tol = ParseOrDefault(json, "goal_yaw_tol", p.goal_yaw_tol);
    p.use_dubins_or_rs = ParseOrDefault(json, "use_dubins_or_rs", p.use_dubins_or_rs);
    p.turning_check_length = ParseOrDefault(json, "turning_check_length", p.turning_check_length);
    p.turning_check_width_scale = ParseOrDefault(json, "turning_check_width_scale", p.turning_check_width_scale);
    p.turning_check_enable = ParseOrDefault(json, "turning_check_enable", p.turning_check_enable);
    p.use_unload_shape_in_rs = ParseOrDefault(json, "use_unload_shape_in_rs", p.use_unload_shape_in_rs);
    p.turning_extend_length = ParseOrDefault(json, "turning_extend_length", p.turning_extend_length);
    p.esdf_vis_max_dist = ParseOrDefault(json, "esdf_vis_max_dist", p.esdf_vis_max_dist);
    p.esdf_pub_hz = ParseOrDefault(json, "esdf_pub_hz", p.esdf_pub_hz);
    p.esdf_use_costmap = ParseOrDefault(json, "esdf_use_costmap", p.esdf_use_costmap);
    p.esdf_resolution_override = ParseOrDefault(json, "esdf_resolution_override", p.esdf_resolution_override);
    p.costmap_occ_threshold = ParseOrDefault(json, "costmap_occ_threshold", p.costmap_occ_threshold);
    p.costmap_unknown_is_obstacle = ParseOrDefault(json, "costmap_unknown_is_obstacle", p.costmap_unknown_is_obstacle);

    // Legacy rectangle helpers mapped into turning check parameters if present.
    p.turning_check_length = ParseOrDefault(json, "legacy_turning_clear_length", p.turning_check_length);
    p.turning_check_width_scale = ParseOrDefault(json, "legacy_turning_clear_width_scale", p.turning_check_width_scale);

    return true;
  }

} // namespace jhzx::center_articulation_planner
