#include "path_planner/costmap_publisher_node.hpp"

namespace path_planner
{

CostmapPublisherNode::CostmapPublisherNode(const rclcpp::NodeOptions & options)
: Node("core_costmap_publisher_node", options)
{
  global_map_topic_ =
    declare_parameter<std::string>("global_map_topic", "/map");
  local_costmap_topic_ =
    declare_parameter<std::string>("local_costmap_topic", "/local_costmap");
  global_frame_id_ = declare_parameter<std::string>("global_frame_id", "map");
  local_frame_id_ = declare_parameter<std::string>("local_frame_id", "map");

  global_width_ = declare_parameter<int>("global_width", 100);
  global_height_ = declare_parameter<int>("global_height", 100);
  global_resolution_ = declare_parameter<double>("global_resolution", 0.05);
  global_origin_x_ = declare_parameter<double>("global_origin_x", -2.5);
  global_origin_y_ = declare_parameter<double>("global_origin_y", -2.5);

  local_width_ = declare_parameter<int>("local_width", 40);
  local_height_ = declare_parameter<int>("local_height", 40);
  local_resolution_ = declare_parameter<double>("local_resolution", 0.05);
  local_origin_x_ = declare_parameter<double>("local_origin_x", -1.0);
  local_origin_y_ = declare_parameter<double>("local_origin_y", -1.0);

  global_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    global_map_topic_, rclcpp::QoS(1).transient_local().reliable());

  local_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, rclcpp::QoS(1).reliable());

  // One-shot timer to publish after construction completes
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CostmapPublisherNode::publishCostmaps, this));

  RCLCPP_INFO(get_logger(), "Costmap Publisher Node initialized");
}

void CostmapPublisherNode::publishCostmaps()
{
  // Cancel timer so this only fires once
  publish_timer_->cancel();

  auto global_map = buildGlobalMap();
  global_map_pub_->publish(global_map);
  RCLCPP_INFO(
    get_logger(), "Published global costmap (%d x %d, %.3f m/cell)",
    global_width_, global_height_, global_resolution_);

  auto local_costmap = buildLocalCostmap();
  local_costmap_pub_->publish(local_costmap);
  RCLCPP_INFO(
    get_logger(), "Published local costmap (%d x %d, %.3f m/cell)",
    local_width_, local_height_, local_resolution_);
}

nav_msgs::msg::OccupancyGrid CostmapPublisherNode::buildGlobalMap() const
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = now();
  map.header.frame_id = global_frame_id_;
  map.info.resolution = static_cast<float>(global_resolution_);
  map.info.width = static_cast<uint32_t>(global_width_);
  map.info.height = static_cast<uint32_t>(global_height_);
  map.info.origin.position.x = global_origin_x_;
  map.info.origin.position.y = global_origin_y_;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;

  // Initialize all cells as free (0)
  map.data.assign(global_width_ * global_height_, 0);

  // Add boundary walls (1-cell thick border)
  fillRect(map, 0, 0, global_width_ - 1, 0, 100);  // bottom
  fillRect(
    map, 0, global_height_ - 1, global_width_ - 1, global_height_ - 1,
    100);                                           // top
  fillRect(map, 0, 0, 0, global_height_ - 1, 100);  // left
  fillRect(
    map, global_width_ - 1, 0, global_width_ - 1, global_height_ - 1,
    100);         // right

  // Add a few rectangular obstacles inside the map
  // Obstacle 1: horizontal bar in the upper-left area
  fillRect(map, 20, 60, 40, 62, 100);

  // Obstacle 2: vertical bar in the center-right area
  fillRect(map, 70, 30, 72, 55, 100);

  // Obstacle 3: small block near center
  fillRect(map, 45, 45, 55, 50, 100);

  return map;
}

nav_msgs::msg::OccupancyGrid CostmapPublisherNode::buildLocalCostmap() const
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = now();
  map.header.frame_id = local_frame_id_;
  map.info.resolution = static_cast<float>(local_resolution_);
  map.info.width = static_cast<uint32_t>(local_width_);
  map.info.height = static_cast<uint32_t>(local_height_);
  map.info.origin.position.x = local_origin_x_;
  map.info.origin.position.y = local_origin_y_;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;

  // All cells free (0) — plain static dummy
  map.data.assign(local_width_ * local_height_, 0);

  return map;
}

void CostmapPublisherNode::fillRect(
  nav_msgs::msg::OccupancyGrid & map, int x0,
  int y0, int x1, int y1,
  int8_t value) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  for (int y = y0; y <= y1 && y < height; ++y) {
    for (int x = x0; x <= x1 && x < width; ++x) {
      map.data[y * width + x] = value;
    }
  }
}

}  // namespace path_planner
