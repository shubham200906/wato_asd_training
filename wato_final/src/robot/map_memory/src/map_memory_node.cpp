#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode() : Node("map_memory"), last_x(0.0), last_y(0.0), last_theta(0.0), distance_threshold(5.0) {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y, last_theta;
    const double distance_threshold;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf2::getYaw(msg->pose.pose.orientation);

        double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

        if (distance >= distance_threshold) {
            last_x = x;
            last_y = y;
            last_theta = theta;
            should_update_map_ = true;
        }
    }

    void updateMap() {
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
        }
    }

    void integrateCostmap() {
        for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
            int x_local = i % latest_costmap_.info.width;
            int y_local = i / latest_costmap_.info.width;

            double x_global = last_x + (x_local * cos(last_theta) - y_local * sin(last_theta));
            double y_global = last_y + (x_local * sin(last_theta) + y_local * cos(last_theta));

            int global_x = (x_global - global_map_.info.origin.position.x) / 0.1;
            int global_y = (y_global - global_map_.info.origin.position.y) / 0.1;

            if (global_x >= 0 && global_y >= 0 && global_x < global_map_.info.width && global_y < global_map_.info.height) {
                if (latest_costmap_.data[i] != -1) {
                    global_map_.data[global_y * global_map_.info.width + global_x] = std::max(
                        global_map_.data[global_y * global_map_.info.width + global_x],
                        latest_costmap_.data[i]
                    );
                }
            }
        }
    }

    nav_msgs::msg::OccupancyGrid latest_costmap_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}