#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}
 
// Define the timer to publish a message every 500ms
/*void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}*/

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int arr[300][300] = {0};

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            double x = range*std::cos(angle);
            double y = range*std::sin(angle);

            x_grid = static_cast<int>((x - x_grid) / 0.1);
            y_grid = static_cast<int>((y - y_grid) / 0.1);

            if(x_grid > 0 && y_grid > 0 && x_grid < 300 && y_grid < 300) {
              arr[x_grid][y_grid] = 100;
            }
        }
    }

    int radius = 10;
    int max_cost = 255;

    for(int dx = -radius; dx < radius; dx++) {
      for(int dy = -radius; dy < radius; dy++) {
        int x = dx + radius;
        int y = dy + radius;
        if(x < 0 || y < 0 || x >= 300 || y >= 300) {
          continue;
        }

        int distance = std::sqrt(dx * dx + dy * dy, 2) * 0.1;
        if(distance <= radius) {
          int cost = max_cost * (1 - (distance / radius));
          arr[x_grid][y_grid] = std::max(arr[x_grid][y_grid], cost);
        }
      }
    }

    publishCostmap(arr);
}

void publishCostmap(int arr[300][300]) {
    nav_msgs::msg::OccupancyGrid costmap_msg;

    // Set the header (frame_id and timestamp)
    costmap_msg.header.frame_id = "map";
    costmap_msg.header.stamp = rclcpp::Clock().now();

    costmap_msg.info.resolution = 0.1;  // 10 cm per cell
    costmap_msg.info.width = 300;        // Grid width
    costmap_msg.info.height = 300;       // Grid height
    costmap_msg.info.origin.position.x = 0.0;  // Origin of the grid
    costmap_msg.info.origin.position.y = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;  // No rotation

    for (int i = 0; i < 300; ++i) {
        for (int j = 0; j < 300; ++j) {
            int index = i + 300 * j;
            costmap_msg.data[index] = arr[i][j];
        }
    }
    costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}