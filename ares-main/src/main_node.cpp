#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarReader : public rclcpp::Node
{
public:
    LidarReader()
        : Node("lidar_reader")
    {
        // Subscribe to the bridged LiDAR topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10,
            std::bind(&LidarReader::lidarCallback, this, std::placeholders::_1)
        );
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received LiDAR cloud with %u points",
            msg->width * msg->height
        );

        // Here you can process the raw point cloud data
        // For example, iterate over points or convert to PCL
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  printf("Starting ares-main\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
