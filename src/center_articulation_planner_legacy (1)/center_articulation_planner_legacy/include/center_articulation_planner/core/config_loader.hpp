#pragma once

#include <string>

#include <global_planner/global_planner.hpp>

namespace jhzx::center_articulation_planner
{

  /**
   * Load vehicle shapes and planner parameters from a JSON-like config file.
   * The parser is lightweight and tolerant of '//' comments as used in params.json.
   *
   * @param path          Absolute path to params.json
   * @param shape_normal  Output normal driving shape
   * @param shape_unload  Output unload shape (callers may prefill defaults)
   * @param cfg           Output planner config (callers may prefill defaults)
   * @param err           Error message when returning false
   * @return true on success, false on failure to read/parse
   */
  bool LoadPlannerConfig(const std::string &path,
                         global_planner::types::VehicleShape &shape_normal,
                         global_planner::types::VehicleShape &shape_unload,
                         global_planner::GlobalPlanner::Config &cfg,
                         std::string &err);

} // namespace jhzx::center_articulation_planner
