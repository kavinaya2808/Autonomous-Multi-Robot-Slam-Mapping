// frontier_explorer: finds frontiers on the merged /map and (optionally)
// sends robots to them through Nav2.
//
//   explore:=false  Phase 1: only detect and draw frontiers in RViz.
//   explore:=true   Phase 2+: also drive the listed robots to frontiers.
//
// Loop (once per second):
//   1. detect frontiers on the latest merged map (frontier_detector.cpp)
//   2. publish RViz markers
//   3. check every robot's current goal (timeout? frontier already explored?)
//   4. coordinated assignment: among ALL free robots and ALL frontiers, take the
//      best (robot, frontier) pair, send that goal, then repeat for the rest.
//      Frontiers close to a goal another robot already has are skipped, so the
//      robots spread out instead of chasing the same frontier.
//      (coordinated:=false turns that off, for comparison experiments.)
//   5. log metrics (CSV) every cycle and save the merged map every few seconds
//      and when exploration is complete.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mrm_exploration/frontier_detector.hpp"
#include "mrm_exploration/map_io.hpp"

namespace mx = mrm_exploration;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using nav_msgs::msg::OccupancyGrid;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct Point2
{
  double x{0.0};
  double y{0.0};
};

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Everything the explorer remembers about one robot.
struct RobotState
{
  std::string name;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client;
  bool busy{false};              // a goal is active
  Point2 goal;                   // where we sent it (world frame)
  rclcpp::Time goal_start;
  GoalHandleNav::SharedPtr handle;
  uint64_t seq{0};               // increases with every goal; old callbacks are ignored
  std::vector<Point2> failed;    // spots THIS robot could not reach (another robot may)

  // Metrics
  bool has_last_pose{false};
  Point2 last_pose;
  double travelled{0.0};         // metres driven since start
  int goals_sent{0};
  int goals_failed{0};
};

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer")
  {
    // --- Parameters ---------------------------------------------------------
    const auto robots_csv = declare_parameter<std::string>("robots", "robot1,robot2");
    explore_ = declare_parameter<bool>("explore", false);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    const double rate_hz = declare_parameter<double>("update_rate", 1.0);
    min_frontier_size_ = declare_parameter<double>("min_frontier_size", 0.4);
    goal_clearance_ = declare_parameter<double>("goal_clearance", 0.3);
    gain_weight_ = declare_parameter<double>("gain_weight", 1.0);
    distance_weight_ = declare_parameter<double>("distance_weight", 1.0);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 60.0);
    blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.5);
    frontier_gone_radius_ = declare_parameter<double>("frontier_gone_radius", 0.5);
    separation_radius_ = declare_parameter<double>("separation_radius", 1.0);
    params_.free_threshold = declare_parameter<int>("free_threshold", 25);
    params_.occupied_threshold = declare_parameter<int>("occupied_threshold", 65);
    coordinated_ = declare_parameter<bool>("coordinated", true);
    const auto output_dir = declare_parameter<std::string>("output_dir", "");
    const auto run_name = declare_parameter<std::string>("run_name", "");
    save_period_ = declare_parameter<double>("save_period", 30.0);
    min_goal_distance_ = declare_parameter<double>("min_goal_distance", 0.5);
    visit_grace_ = declare_parameter<double>("visit_grace", 5.0);

    // --- Robots: one Nav2 action client each ---------------------------------
    // "robot1,robot2" -> ["robot1", "robot2"]
    std::stringstream ss(robots_csv);
    std::string name;
    std::vector<std::string> names;
    while (std::getline(ss, name, ',')) {
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
      if (!name.empty()) {
        names.push_back(name);
      }
    }
    // Reserve first: lambdas below refer to robots by index, and the vector
    // must never reallocate while they exist.
    robots_.reserve(names.size());
    for (const auto & n : names) {
      RobotState r;
      r.name = n;
      r.client = rclcpp_action::create_client<NavigateToPose>(this, "/" + n + "/navigate_to_pose");
      r.goal_start = now();
      robots_.push_back(r);
      RCLCPP_INFO(get_logger(), "Robot: %s", n.c_str());
    }

    // --- TF, map, markers, timer --------------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<OccupancyGrid>(
      map_topic, latched, [this](OccupancyGrid::ConstSharedPtr msg) {map_ = msg;});

    marker_pub_ = create_publisher<MarkerArray>("/frontiers", 1);

    setupOutput(output_dir, run_name);

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      [this]() {step();});

    RCLCPP_INFO(
      get_logger(), "Frontier explorer started (%s%s)",
      explore_ ? "EXPLORING" : "detection only - robots will not move",
      explore_ ? (coordinated_ ? ", coordinated" : ", NOT coordinated") : "");
  }

private:
  // ======================================================================== main loop
  void step()
  {
    if (!map_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for the merged map...");
      return;
    }
    const OccupancyGrid & map = *map_;
    const double res = map.info.resolution;
    const mx::Grid grid{
      static_cast<int>(map.info.width), static_cast<int>(map.info.height), map.data.data()};

    // Metres -> cells for this map's resolution.
    params_.min_frontier_cells = std::max(1, static_cast<int>(std::round(min_frontier_size_ / res)));
    params_.clearance_cells = std::max(1, static_cast<int>(std::ceil(goal_clearance_ / res)));

    const std::vector<mx::Frontier> frontiers = mx::findFrontiers(grid, params_);
    publishMarkers(map, frontiers);

    // Where is every robot? Also accumulate how far each one has driven.
    std::vector<Point2> poses(robots_.size());
    std::vector<bool> have_pose(robots_.size(), false);
    for (size_t i = 0; i < robots_.size(); ++i) {
      RobotState & r = robots_[i];
      if (robotPosition(r.name, poses[i])) {
        have_pose[i] = true;
        if (r.has_last_pose) {
          r.travelled += distance(poses[i], r.last_pose);
        }
        r.last_pose = poses[i];
        r.has_last_pose = true;
      }
    }

    logMetrics(grid, res, frontiers.size());
    if ((now() - last_save_).seconds() >= save_period_) {
      saveMapNow("map");
    }
    reviewVisitedSpots(map, frontiers);

    if (!explore_) {
      return;
    }

    // --- 1) Check every robot's current goal; collect the robots that are free.
    bool any_waiting = false;            // a robot or its Nav2 is not ready yet
    std::vector<size_t> idle;            // indices into robots_
    std::vector<Point2> idle_pose;
    for (size_t i = 0; i < robots_.size(); ++i) {
      RobotState & r = robots_[i];
      if (!have_pose[i]) {
        any_waiting = true;
        continue;
      }
      const Point2 pose = poses[i];

      if (r.busy) {
        const double elapsed = (now() - r.goal_start).seconds();
        if (elapsed > goal_timeout_) {
          RCLCPP_WARN(
            get_logger(), "[%s] goal (%.2f, %.2f) took more than %.0f s: giving up on it",
            r.name.c_str(), r.goal.x, r.goal.y, goal_timeout_);
          r.failed.push_back(r.goal);    // only THIS robot avoids it from now on
          ++r.goals_failed;
          cancelGoal(r);
        } else if (!frontierNear(map, frontiers, r.goal, frontier_gone_radius_)) {
          // The frontier we were driving to has been mapped already (by this
          // robot's lidar from a distance, or by the other robot). No need to
          // finish the trip: choose the next frontier now.
          RCLCPP_INFO(
            get_logger(), "[%s] frontier at (%.2f, %.2f) is already explored, re-planning",
            r.name.c_str(), r.goal.x, r.goal.y);
          r.busy = false;   // a new goal below replaces (preempts) the old one
        } else {
          continue;         // still on its way: leave it alone
        }
      }

      if (!r.client->action_server_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "[%s] Nav2 is not ready (is navigation running?)",
          r.name.c_str());
        any_waiting = true;
        continue;
      }
      idle.push_back(i);
      idle_pose.push_back(pose);
    }

    // --- 2) One BFS per free robot: walking distance to every cell.
    std::vector<std::vector<int>> dist(idle.size());
    for (size_t k = 0; k < idle.size(); ++k) {
      const mx::Cell start = worldToCell(map, idle_pose[k]);
      if (grid.inside(start.x, start.y)) {
        dist[k] = mx::pathDistances(grid, start, params_);
      } else {
        any_waiting = true;   // robot is outside the map (map still growing)
      }
    }

    // --- 3) Coordinated (global greedy) assignment.
    // Repeatedly take the single best (robot, frontier) pair over ALL free
    // robots. After a robot gets a goal it counts as busy, so the
    // separation rule keeps the next robot away from that goal.
    std::vector<bool> assigned(idle.size(), false);
    bool any_assigned = false;
    while (true) {
      double best_score = -std::numeric_limits<double>::infinity();
      int best_k = -1;
      const mx::Frontier * best_f = nullptr;
      mx::Cell best_cell;
      double best_dist_m = 0.0;

      for (size_t k = 0; k < idle.size(); ++k) {
        if (assigned[k] || dist[k].empty()) {
          continue;
        }
        const size_t robot = idle[k];
        for (const auto & f : frontiers) {
          // Pick a goal cell inside this frontier that suits THIS robot. A
          // blacklisted or too-close spot no longer rules out the whole
          // frontier: we just use another cell of it.
          mx::Cell cell;
          if (!pickTarget(map, grid, f, robot, idle_pose[k], dist[k], cell)) {
            continue;
          }
          const int d = dist[k][grid.index(cell.x, cell.y)];
          const double size_m = static_cast<double>(f.cells.size()) * res;
          const double dist_m = static_cast<double>(d) * res;
          const double score = gain_weight_ * size_m - distance_weight_ * dist_m;
          if (score > best_score) {
            best_score = score;
            best_k = static_cast<int>(k);
            best_f = &f;
            best_cell = cell;
            best_dist_m = dist_m;
          }
        }
      }
      if (best_k < 0) {
        break;   // no robot can use any remaining frontier
      }

      const size_t robot = idle[static_cast<size_t>(best_k)];
      const Point2 target = cellToWorld(map, best_cell);
      RCLCPP_INFO(
        get_logger(), "[%s] -> frontier at (%.2f, %.2f): size %.2f m, path %.2f m, score %.2f",
        robots_[robot].name.c_str(), target.x, target.y,
        static_cast<double>(best_f->cells.size()) * res, best_dist_m, best_score);
      sendGoal(robot, idle_pose[static_cast<size_t>(best_k)], target);
      assigned[static_cast<size_t>(best_k)] = true;
      any_assigned = true;
    }

    // Free robots that got nothing: stop any old goal they were still driving.
    for (size_t k = 0; k < idle.size(); ++k) {
      RobotState & r = robots_[idle[k]];
      if (!assigned[k] && r.handle) {
        cancelGoal(r);
      }
    }

    // --- 4) Finished?
    bool any_busy = false;
    for (const auto & r : robots_) {
      any_busy = any_busy || r.busy;
    }
    // Spots visited in the last few seconds are still being checked: the map
    // needs a moment to catch up, so we can't call it finished yet.
    any_waiting = any_waiting || !recent_visits_.empty();
    if (!any_busy && !any_assigned && !any_waiting) {
      if (!finished_logged_) {
        RCLCPP_INFO(
          get_logger(), "Exploration complete: no reachable frontiers left "
          "(%zu frontiers visible, %zu spots blacklisted)", frontiers.size(), blacklist_.size());
        finished_logged_ = true;
        saveMapNow("map_final");
        writeSummary(grid, res);
      }
    } else {
      finished_logged_ = false;
    }
  }

  // ======================================================================== Nav2 action client
  void sendGoal(size_t index, const Point2 & pose, const Point2 & target)
  {
    RobotState & r = robots_[index];
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = world_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    // Face the direction of travel (robot -> frontier): yaw -> quaternion around z.
    const double yaw = std::atan2(target.y - pose.y, target.x - pose.x);
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    const uint64_t seq = ++r.seq;
    ++r.goals_sent;
    r.busy = true;
    r.goal = target;
    r.goal_start = now();
    r.handle.reset();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    // Called once Nav2 accepts or rejects the goal.
    options.goal_response_callback =
      [this, index, seq](GoalHandleNav::SharedPtr handle) {
        RobotState & robot = robots_[index];
        if (seq != robot.seq) {
          return;   // an older goal; we have moved on
        }
        if (!handle) {
          RCLCPP_WARN(get_logger(), "[%s] Nav2 rejected the goal", robot.name.c_str());
          robot.failed.push_back(robot.goal);
          ++robot.goals_failed;
          robot.busy = false;
          return;
        }
        robot.handle = handle;
      };

    // Called once when the goal finishes (succeeded, aborted or cancelled).
    options.result_callback =
      [this, index, seq](const GoalHandleNav::WrappedResult & result) {
        RobotState & robot = robots_[index];
        if (seq != robot.seq) {
          return;   // result of a goal we already replaced
        }
        robot.busy = false;
        robot.handle.reset();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "[%s] reached the frontier", robot.name.c_str());
            // Don't judge yet: SLAM and the merger need a few seconds to add
            // what the robot sees from here. reviewVisitedSpots() decides later.
            recent_visits_.push_back({robot.goal, now()});
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(
              get_logger(), "[%s] Nav2 could not reach (%.2f, %.2f): it will try elsewhere",
              robot.name.c_str(), robot.goal.x, robot.goal.y);
            // Only this robot avoids the spot: the failure may be about this
            // robot (stuck, bad angle), so the other robot may still go there.
            robot.failed.push_back(robot.goal);
            ++robot.goals_failed;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            break;
          default:
            break;
        }
      };

    r.client->async_send_goal(goal, options);
  }

  void cancelGoal(RobotState & r)
  {
    if (r.handle) {
      r.client->async_cancel_goal(r.handle);
    }
    ++r.seq;          // ignore any callback from the cancelled goal
    r.busy = false;
    r.handle.reset();
  }

  // ======================================================================== helpers
  bool robotPosition(const std::string & robot, Point2 & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        world_frame_, robot + "/base_footprint", tf2::TimePointZero);
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "[%s] no pose yet: %s", robot.c_str(), ex.what());
      return false;
    }
  }

  // The merged map is axis-aligned with the world frame (map_merger builds it
  // that way), so converting is just origin + index * resolution.
  static Point2 cellToWorld(const OccupancyGrid & map, const mx::Cell & c)
  {
    const double res = map.info.resolution;
    return {map.info.origin.position.x + (c.x + 0.5) * res,
      map.info.origin.position.y + (c.y + 0.5) * res};
  }

  static mx::Cell worldToCell(const OccupancyGrid & map, const Point2 & p)
  {
    const double res = map.info.resolution;
    return {static_cast<int>(std::floor((p.x - map.info.origin.position.x) / res)),
      static_cast<int>(std::floor((p.y - map.info.origin.position.y) / res))};
  }

  // Visited-but-unexplorable spots (everyone) + spots this robot failed at.
  bool isBlacklistedFor(size_t index, const Point2 & p) const
  {
    if (isBlacklisted(p)) {
      return true;
    }
    for (const auto & v : recent_visits_) {
      if (distance(p, v.spot) < blacklist_radius_) {
        return true;   // just visited: wait for the map before going back
      }
    }
    for (const auto & b : robots_[index].failed) {
      if (distance(p, b) < blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  bool isBlacklisted(const Point2 & p) const
  {
    for (const auto & b : blacklist_) {
      if (distance(p, b) < blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  bool nearOtherRobotsGoal(size_t index, const Point2 & p) const
  {
    if (!coordinated_) {
      return false;   // experiment mode: robots ignore each other's goals
    }
    for (size_t j = 0; j < robots_.size(); ++j) {
      if (j != index && robots_[j].busy && distance(p, robots_[j].goal) < separation_radius_) {
        return true;
      }
    }
    return false;
  }

  static bool frontierNear(
    const OccupancyGrid & map, const std::vector<mx::Frontier> & frontiers,
    const Point2 & p, double radius)
  {
    for (const auto & f : frontiers) {
      for (const auto & c : f.cells) {
        if (distance(cellToWorld(map, c), p) < radius) {
          return true;
        }
      }
    }
    return false;
  }

  // Choose the goal cell inside frontier f for one robot: prefer cells with
  // room around them (clearance), skip cells that are blacklisted, too close to
  // the robot, near another robot's goal or unreachable; among the rest take
  // the one nearest the frontier's middle (centroid) for the best view.
  bool pickTarget(
    const OccupancyGrid & map, const mx::Grid & grid, const mx::Frontier & f,
    size_t robot, const Point2 & pose, const std::vector<int> & dist, mx::Cell & out) const
  {
    for (int pass = 0; pass < 2; ++pass) {   // pass 0: clear cells only, pass 1: any cell
      bool found = false;
      long long best_d2 = std::numeric_limits<long long>::max();
      for (size_t i = 0; i < f.cells.size(); ++i) {
        const mx::Cell & c = f.cells[i];
        const int cl = f.clearance[i];
        if (pass == 0 && cl >= 0 && cl < params_.clearance_cells) {
          continue;
        }
        if (dist[grid.index(c.x, c.y)] < 0) {
          continue;   // not reachable through known free space
        }
        const Point2 w = cellToWorld(map, c);
        if (distance(w, pose) < min_goal_distance_) {
          continue;   // right next to the robot: its lidar covers this anyway
        }
        if (isBlacklistedFor(robot, w) || nearOtherRobotsGoal(robot, w)) {
          continue;
        }
        const long long dx = c.x - f.centroid.x;
        const long long dy = c.y - f.centroid.y;
        const long long d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          out = c;
          found = true;
        }
      }
      if (found) {
        return true;
      }
    }
    return false;
  }

  // A few seconds after a robot arrives somewhere, look again: if the frontier
  // there is gone, all good. If it is STILL there, the lidar cannot see into it
  // from this spot (too far, under furniture...), so nobody should come back.
  void reviewVisitedSpots(const OccupancyGrid & map, const std::vector<mx::Frontier> & frontiers)
  {
    const rclcpp::Time t = now();
    auto it = recent_visits_.begin();
    while (it != recent_visits_.end()) {
      if ((t - it->when).seconds() < visit_grace_) {
        ++it;
        continue;
      }
      if (frontierNear(map, frontiers, it->spot, frontier_gone_radius_)) {
        RCLCPP_INFO(
          get_logger(), "Frontier at (%.2f, %.2f) is still unexplored after a visit: "
          "blacklisting it", it->spot.x, it->spot.y);
        blacklist_.push_back(it->spot);
      }
      it = recent_visits_.erase(it);
    }
  }

  // ======================================================================== results: files on disk
  // One folder per run: <output_dir>/<run_name>/ with
  //   metrics.csv     one line per second: time, explored area, frontiers, per-robot stats
  //   map.pgm/.yaml   latest merged map (overwritten every save_period seconds)
  //   map_final.*     the map at "Exploration complete"
  //   summary.txt     totals at the end
  void setupOutput(std::string output_dir, std::string run_name)
  {
    if (output_dir.empty()) {
      const char * home = std::getenv("HOME");
      output_dir = std::string(home != nullptr ? home : ".") + "/mrm_ws/exploration_runs";
    }
    if (run_name.empty() || run_name == "auto") {
      const std::time_t t = std::time(nullptr);
      std::ostringstream name;
      name << "run_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
      run_name = name.str();
    }
    run_dir_ = output_dir + "/" + run_name;

    std::error_code ec;
    std::filesystem::create_directories(run_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Cannot create %s: %s", run_dir_.c_str(), ec.message().c_str());
      return;
    }

    csv_.open(run_dir_ + "/metrics.csv");
    csv_ << "time_s,explored_m2,frontiers";
    for (const auto & r : robots_) {
      csv_ << ',' << r.name << "_travelled_m," << r.name << "_goals," << r.name << "_failed";
    }
    csv_ << std::endl;
    RCLCPP_INFO(get_logger(), "Saving results in %s", run_dir_.c_str());
  }

  void logMetrics(const mx::Grid & grid, double res, size_t frontier_count)
  {
    if (!started_) {
      start_time_ = now();
      last_save_ = start_time_;
      started_ = true;
    }
    if (!csv_.is_open()) {
      return;
    }
    const double t = (now() - start_time_).seconds();
    const double explored_m2 = static_cast<double>(mx::countKnownCells(grid)) * res * res;
    csv_ << std::fixed << std::setprecision(2) << t << ',' << explored_m2 << ',' << frontier_count;
    for (const auto & r : robots_) {
      csv_ << ',' << r.travelled << ',' << r.goals_sent << ',' << r.goals_failed;
    }
    csv_ << std::endl;   // endl flushes: the file is complete even if we crash
  }

  void saveMapNow(const std::string & name)
  {
    last_save_ = now();
    if (!map_ || run_dir_.empty()) {
      return;
    }
    const OccupancyGrid & map = *map_;
    const mx::Grid grid{
      static_cast<int>(map.info.width), static_cast<int>(map.info.height), map.data.data()};
    mx::MapFileInfo info;
    info.resolution = map.info.resolution;
    info.origin_x = map.info.origin.position.x;
    info.origin_y = map.info.origin.position.y;
    std::string error;
    if (mx::saveMap(grid, info, run_dir_ + "/" + name, &error)) {
      RCLCPP_INFO(get_logger(), "Saved %s/%s.pgm", run_dir_.c_str(), name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Could not save the map: %s", error.c_str());
    }
  }

  void writeSummary(const mx::Grid & grid, double res)
  {
    const double t = started_ ? (now() - start_time_).seconds() : 0.0;
    const double explored_m2 = static_cast<double>(mx::countKnownCells(grid)) * res * res;
    std::ostringstream s;
    s << std::fixed << std::setprecision(2)
      << "robots: " << robots_.size() << (coordinated_ ? " (coordinated)" : " (not coordinated)")
      << "\ntime_to_complete_s: " << t
      << "\nexplored_m2: " << explored_m2 << "\n";
    for (const auto & r : robots_) {
      s << r.name << ": travelled_m " << r.travelled << ", goals " << r.goals_sent
        << ", failed " << r.goals_failed << "\n";
    }
    RCLCPP_INFO(get_logger(), "Run summary:\n%s", s.str().c_str());
    if (!run_dir_.empty()) {
      std::ofstream(run_dir_ + "/summary.txt") << s.str();
    }
  }

  // ======================================================================== RViz markers
  void publishMarkers(const OccupancyGrid & map, const std::vector<mx::Frontier> & frontiers)
  {
    MarkerArray array;
    const auto stamp = now();
    const double res = map.info.resolution;

    Marker clear;
    clear.action = Marker::DELETEALL;   // wipe last cycle's markers first
    array.markers.push_back(clear);

    // All frontier cells: small yellow squares.
    Marker cells;
    cells.header.frame_id = world_frame_;
    cells.header.stamp = stamp;
    cells.ns = "frontier_cells";
    cells.id = 0;
    cells.type = Marker::CUBE_LIST;
    cells.action = Marker::ADD;
    cells.pose.orientation.w = 1.0;
    cells.scale.x = res;
    cells.scale.y = res;
    cells.scale.z = 0.02;
    cells.color.r = 1.0f;
    cells.color.g = 0.85f;
    cells.color.b = 0.0f;
    cells.color.a = 0.9f;

    // One sphere per frontier at its target: green = clear, orange = tight spot.
    Marker targets;
    targets.header = cells.header;
    targets.ns = "frontier_targets";
    targets.id = 1;
    targets.type = Marker::SPHERE_LIST;
    targets.action = Marker::ADD;
    targets.pose.orientation.w = 1.0;
    targets.scale.x = targets.scale.y = targets.scale.z = 0.15;

    for (const auto & f : frontiers) {
      for (const auto & c : f.cells) {
        const Point2 w = cellToWorld(map, c);
        geometry_msgs::msg::Point p;
        p.x = w.x;
        p.y = w.y;
        p.z = 0.02;
        cells.points.push_back(p);
      }
      const Point2 t = cellToWorld(map, f.target);
      geometry_msgs::msg::Point p;
      p.x = t.x;
      p.y = t.y;
      p.z = 0.1;
      targets.points.push_back(p);
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0f;
      if (isBlacklisted(t)) {
        color.r = color.g = color.b = 0.5f;          // grey: blacklisted
      } else if (f.target_is_clear) {
        color.g = 0.8f;                              // green
      } else {
        color.r = 1.0f;
        color.g = 0.5f;                              // orange
      }
      targets.colors.push_back(color);
    }
    if (!cells.points.empty()) {
      array.markers.push_back(cells);
      array.markers.push_back(targets);
    }

    // Current goal of each robot: a big sphere (robot1 red, others blue).
    for (size_t i = 0; i < robots_.size(); ++i) {
      if (!robots_[i].busy) {
        continue;
      }
      Marker goal;
      goal.header = cells.header;
      goal.ns = "robot_goals";
      goal.id = static_cast<int>(i);
      goal.type = Marker::SPHERE;
      goal.action = Marker::ADD;
      goal.pose.position.x = robots_[i].goal.x;
      goal.pose.position.y = robots_[i].goal.y;
      goal.pose.position.z = 0.15;
      goal.pose.orientation.w = 1.0;
      goal.scale.x = goal.scale.y = goal.scale.z = 0.3;
      goal.color.a = 0.9f;
      if (robots_[i].name == "robot1") {
        goal.color.r = 0.9f;
      } else {
        goal.color.b = 0.9f;
        goal.color.g = 0.4f;
      }
      array.markers.push_back(goal);
    }

    marker_pub_->publish(array);
  }

  // ======================================================================== members
  bool explore_{false};
  std::string world_frame_;
  double min_frontier_size_{0.4};
  double goal_clearance_{0.3};
  double gain_weight_{1.0};
  double distance_weight_{1.0};
  double goal_timeout_{60.0};
  double blacklist_radius_{0.5};
  double frontier_gone_radius_{0.5};
  double separation_radius_{1.0};
  bool coordinated_{true};
  double save_period_{30.0};
  double min_goal_distance_{0.5};
  double visit_grace_{5.0};

  struct Visit
  {
    Point2 spot;
    rclcpp::Time when;
  };
  std::vector<Visit> recent_visits_;
  mx::DetectorParams params_;

  // Results
  std::string run_dir_;
  std::ofstream csv_;
  bool started_{false};
  rclcpp::Time start_time_;
  rclcpp::Time last_save_;

  std::vector<RobotState> robots_;
  std::vector<Point2> blacklist_;
  bool finished_logged_{false};

  OccupancyGrid::ConstSharedPtr map_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
