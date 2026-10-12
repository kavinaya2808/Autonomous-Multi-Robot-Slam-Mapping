// map_merger: fuses several robots' occupancy grids into one world-frame map.
//
// Inputs
//   /<robot>/map       nav_msgs/OccupancyGrid from each robot's slam_toolbox,
//                      expressed in that robot's own frame (e.g. robot1/map).
//   TF  world -> <robot>/map   where each robot's map sits in the shared world.
//                      (Published by static_transform_publisher in the launch
//                      file, from the robots' known start poses.)
// Output
//   /map               nav_msgs/OccupancyGrid in the "world" frame.
//
// Why not just stitch images? Each SLAM map has its own origin (bottom-left
// corner) and it moves as the map grows. Working in metric world coordinates
// with TF keeps the merged map correctly georeferenced, which the exploration
// node will need later when it turns frontier cells into navigation goals.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

using nav_msgs::msg::OccupancyGrid;

class MapMerger : public rclcpp::Node
{
public:
  MapMerger()
  : Node("map_merger")
  {
    // --- Parameters ---------------------------------------------------------
    // declare_parameter() registers the parameter (so it shows up in
    // `ros2 param list`) and returns its value: the YAML/launch value if one
    // was given, otherwise the default written here.
    robots_ = declare_parameter<std::vector<std::string>>(
      "robots", std::vector<std::string>{"robot1", "robot2"});
    const auto map_topic = declare_parameter<std::string>("robot_map_topic", "map");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    const auto merged_topic = declare_parameter<std::string>("merged_map_topic", "/map");
    const double rate_hz = declare_parameter<double>("publish_rate", 1.0);

    // --- TF -----------------------------------------------------------------
    // The Buffer stores recent transforms; the Listener subscribes to /tf and
    // /tf_static and fills the Buffer in the background. Giving the Buffer our
    // node's clock makes it follow /clock when use_sim_time is true.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- QoS ----------------------------------------------------------------
    // slam_toolbox publishes its map as "transient local" (latched): the last
    // message is kept and handed to anyone who subscribes later. We ask for the
    // same so we get a map immediately, and publish ours the same way so RViz
    // and Nav2 receive it even if they start after us.
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    // --- Subscriptions (one per robot) ---------------------------------------
    for (const auto & robot : robots_) {
      const std::string topic = "/" + robot + "/" + map_topic;
      // The lambda captures `robot` by value so each callback knows which
      // robot it belongs to. We only store the pointer here (cheap); the heavy
      // work happens on the timer. With the default single-threaded executor,
      // callbacks never run at the same time, so no mutex is needed.
      subs_.push_back(create_subscription<OccupancyGrid>(
        topic, latched,
        [this, robot](OccupancyGrid::ConstSharedPtr msg) {latest_maps_[robot] = msg;}));
      RCLCPP_INFO(get_logger(), "Listening for %s", topic.c_str());
    }

    merged_pub_ = create_publisher<OccupancyGrid>(merged_topic, latched);

    // --- Timer --------------------------------------------------------------
    // rclcpp::create_timer with the node clock runs on ROS time, so with
    // use_sim_time it pauses when Unity pauses. (create_wall_timer would use
    // the computer's clock instead.)
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      [this]() {mergeAndPublish();});
  }

private:
  // A robot map that is ready to merge, plus the transform that takes a point
  // in world coordinates into that grid's "cell space" (metres from the
  // grid's bottom-left corner, along the grid's own axes).
  struct Source
  {
    OccupancyGrid::ConstSharedPtr grid;
    tf2::Transform world_to_grid;
  };

  void mergeAndPublish()
  {
    std::vector<Source> sources;
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double resolution = std::numeric_limits<double>::max();

    // 1) For every robot map: find where it sits in the world frame and grow
    //    the merged map's bounding box to cover it.
    for (const auto & [robot, grid] : latest_maps_) {
      if (!grid || grid->info.width == 0 || grid->info.height == 0) {
        continue;
      }

      geometry_msgs::msg::TransformStamped world_to_map_msg;
      try {
        // "Where is the map frame, expressed in world?" TimePointZero means
        // "the latest available"; fine here because these transforms are static.
        world_to_map_msg = tf_buffer_->lookupTransform(
          world_frame_, grid->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Skipping %s: %s", robot.c_str(), ex.what());
        continue;
      }

      // Chain: world -> robot map frame -> grid origin (bottom-left cell corner).
      tf2::Transform world_T_map;
      tf2::fromMsg(world_to_map_msg.transform, world_T_map);
      tf2::Transform map_T_grid;
      tf2::fromMsg(grid->info.origin, map_T_grid);
      const tf2::Transform world_T_grid = world_T_map * map_T_grid;

      // Transform the grid's four corners into world to extend the bounds.
      const double w = grid->info.width * grid->info.resolution;
      const double h = grid->info.height * grid->info.resolution;
      for (const auto & corner : {tf2::Vector3(0, 0, 0), tf2::Vector3(w, 0, 0),
          tf2::Vector3(0, h, 0), tf2::Vector3(w, h, 0)})
      {
        const tf2::Vector3 p = world_T_grid * corner;
        min_x = std::min(min_x, p.x());
        min_y = std::min(min_y, p.y());
        max_x = std::max(max_x, p.x());
        max_y = std::max(max_y, p.y());
      }

      resolution = std::min(resolution, static_cast<double>(grid->info.resolution));
      sources.push_back({grid, world_T_grid.inverse()});
    }

    if (sources.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for robot maps...");
      return;
    }

    // 2) Allocate the merged grid, axis-aligned with the world frame.
    OccupancyGrid merged;
    merged.header.frame_id = world_frame_;
    merged.header.stamp = now();
    merged.info.map_load_time = merged.header.stamp;
    merged.info.resolution = static_cast<float>(resolution);
    merged.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution));
    merged.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution));
    merged.info.origin.position.x = min_x;
    merged.info.origin.position.y = min_y;
    merged.info.origin.orientation.w = 1.0;
    merged.data.assign(static_cast<size_t>(merged.info.width) * merged.info.height, -1);

    // 3) For each merged cell, look up the same world point in every robot
    //    map. Cell values: -1 unknown, 0 free ... 100 occupied. Taking the
    //    maximum means: unknown only if nobody has seen it, and an obstacle
    //    seen by any robot wins over free space (the safe choice for planning).
    for (uint32_t iy = 0; iy < merged.info.height; ++iy) {
      for (uint32_t ix = 0; ix < merged.info.width; ++ix) {
        const tf2::Vector3 p_world(
          min_x + (ix + 0.5) * resolution, min_y + (iy + 0.5) * resolution, 0.0);

        int8_t value = -1;
        for (const auto & src : sources) {
          const tf2::Vector3 p = src.world_to_grid * p_world;
          const auto & info = src.grid->info;
          const auto cx = static_cast<int64_t>(std::floor(p.x() / info.resolution));
          const auto cy = static_cast<int64_t>(std::floor(p.y() / info.resolution));
          if (cx < 0 || cy < 0 || cx >= info.width || cy >= info.height) {
            continue;
          }
          // OccupancyGrid data is row-major: row = y, column = x.
          value = std::max(value, src.grid->data[cy * info.width + cx]);
        }
        merged.data[static_cast<size_t>(iy) * merged.info.width + ix] = value;
      }
    }

    merged_pub_->publish(merged);
    RCLCPP_INFO_ONCE(
      get_logger(), "Publishing merged map (%zu robot maps)", sources.size());
  }

  std::vector<std::string> robots_;
  std::string world_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<rclcpp::Subscription<OccupancyGrid>::SharedPtr> subs_;
  std::map<std::string, OccupancyGrid::ConstSharedPtr> latest_maps_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                   // start the ROS 2 client library
  rclcpp::spin(std::make_shared<MapMerger>());  // run callbacks until Ctrl+C
  rclcpp::shutdown();
  return 0;
}
