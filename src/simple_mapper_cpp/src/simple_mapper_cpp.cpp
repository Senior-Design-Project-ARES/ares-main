#include <cmath>
#include <deque>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class OccupancyMapper : public rclcpp::Node
{
public:
    OccupancyMapper()
    : Node("occupancy_mapper"),
      static_br_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
      dynamic_br_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
    {
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<double>("resolution", 0.02);
        declare_parameter<int>("width", 250);
        declare_parameter<int>("height", 250);
        declare_parameter<double>("max_log", 50.0);
        declare_parameter<double>("min_log", -10.0);
        declare_parameter<double>("range_min", 0.15);
        declare_parameter<double>("publish_rate", 12.0);
        declare_parameter<int>("occ_radius_cells", 1);
        declare_parameter<int>("start_padding_cells", 5);
        declare_parameter<double>("laser_offset_x", 0.0);
        declare_parameter<double>("laser_offset_y", 0.0);

        map_frame_ = get_parameter("map_frame").as_string();
        res_ = get_parameter("resolution").as_double();
        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        max_log_ = get_parameter("max_log").as_double();
        min_log_ = get_parameter("min_log").as_double();
        rmin_ = get_parameter("range_min").as_double();
        double pub_rate = get_parameter("publish_rate").as_double();
        occ_radius_ = get_parameter("occ_radius_cells").as_int();
        start_pad_ = get_parameter("start_padding_cells").as_int();
        laser_offset_x_ = get_parameter("laser_offset_x").as_double();
        laser_offset_y_ = get_parameter("laser_offset_y").as_double();

        log_.assign(width_ * height_, 0.0f);

        origin_x_ = 0.0;
        origin_y_ = 0.0;

        publish_static_frames();

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10,
            std::bind(&OccupancyMapper::on_pose, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&OccupancyMapper::on_scan, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / pub_rate),
            std::bind(&OccupancyMapper::process_and_publish, this));

        RCLCPP_INFO(
            get_logger(),
            "OccupancyMapper ready | grid %dx%d @ %.3f m/cell | range cap %.2f m | "
            "L_OCC=%.1f L_FREE=%.1f | sync tolerance %.0f ms | processing at %.1f Hz",
            width_, height_, res_, HARD_RANGE_MAX, L_OCC, L_FREE,
            MAX_SYNC_DELTA_SEC * 1000.0, pub_rate);
    }

private:
    static constexpr double L_OCC = 1.0;
    static constexpr double L_FREE = 0.1111;
    static constexpr double HARD_RANGE_MAX = 4.0;
    static constexpr double MAX_SYNC_DELTA_SEC = 0.2;

    std::string map_frame_;
    double res_;
    int width_;
    int height_;
    double max_log_;
    double min_log_;
    double rmin_;
    int occ_radius_;
    int start_pad_;
    double laser_offset_x_;
    double laser_offset_y_;
    double origin_x_;
    double origin_y_;
    size_t num_of_scans_processed_ = 0;

    std::vector<float> log_;

    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_buffer_;
    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_br_;

    static double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    static double yaw_from_quat(double qx, double qy, double qz, double qw)
    {
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    std::pair<int, int> world_to_grid(double x, double y) const
    {
        int gx = static_cast<int>(std::floor((x - origin_x_) / res_));
        int gy = static_cast<int>(std::floor((y - origin_y_) / res_));
        return {gx, gy};
    }

    bool in_bounds(int gx, int gy) const
    {
        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }

    std::pair<int, int> clamp_grid(int gx, int gy) const
    {
        gx = std::max(0, std::min(width_ - 1, gx));
        gy = std::max(0, std::min(height_ - 1, gy));
        return {gx, gy};
    }

    int idx(int gx, int gy) const
    {
        return gy * width_ + gx;
    }

    std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) const
    {
        std::vector<std::pair<int, int>> cells;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0;
        int y = y0;

        while (true) {
            cells.emplace_back(x, y);
            if (x == x1 && y == y1) {
                break;
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
        return cells;
    }

    void publish_static_frames()
    {
        auto now = get_clock()->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        geometry_msgs::msg::TransformStamped t1;
        t1.header.stamp = now;
        t1.header.frame_id = "world";
        t1.child_frame_id = "map";
        t1.transform.rotation.w = 1.0;
        transforms.push_back(t1);

        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = now;
        t2.header.frame_id = "base_link";
        t2.child_frame_id = "laser";
        t2.transform.rotation.w = 1.0;
        transforms.push_back(t2);

        static_br_->sendTransform(transforms);
    }

    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_buffer_.push_back(msg);
        while (pose_buffer_.size() > 180) {
            pose_buffer_.pop_front();
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = msg->pose.position.x;
        t.transform.translation.y = msg->pose.position.y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = msg->pose.orientation;
        dynamic_br_->sendTransform(t);
    }

    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // RCLCPP_INFO(
        //     get_logger(),
        //     "Received scan Fwarnwith timestamp %.3f",
        //     stamp_to_sec(scan->header.stamp));

        scan_buffer_.push_back(scan);
        while (scan_buffer_.size() > 10) {
            scan_buffer_.pop_front();
        }
    }

    geometry_msgs::msg::PoseStamped::SharedPtr get_pose_at_time_sec(double query_t)
    {
        if (pose_buffer_.size() < 2) {
            return nullptr;
        }

        geometry_msgs::msg::PoseStamped::SharedPtr before = nullptr;
        geometry_msgs::msg::PoseStamped::SharedPtr after = nullptr;
        double before_t = -std::numeric_limits<double>::infinity();
        double after_t = std::numeric_limits<double>::infinity();
        size_t before_idx = 0;
        size_t after_idx = 0;

        for (int i = 0; i < pose_buffer_.size(); ++i) {
            const auto & pose = pose_buffer_[i];
            double t = stamp_to_sec(pose->header.stamp);
            if (t <= query_t && t > before_t) {
                before_t = t;
                before = pose;
                before_idx = i;
            }
            if (t > query_t && t < after_t) {
                after_t = t;
                after = pose;
                after_idx = i;
            }
        }

        if (!before && !after) {
            RCLCPP_WARN(get_logger(), "No before and after pose found");
            return nullptr;
        }

        if (!before) {
            return (std::abs(after_t - query_t) < MAX_SYNC_DELTA_SEC) ? after : nullptr;
        }

        if (!after) {
            return (std::abs(query_t - before_t) < MAX_SYNC_DELTA_SEC) ? before : nullptr;
        }

        if (std::abs(after_t - query_t) <= std::abs(query_t - before_t)) {
            // RCLCPP_INFO(
            //     get_logger(),
            //     "Matched scan time %.3f to after pose at %.3f (delta %.0f ms) [before delta %.0f ms] index %zu",
            //     query_t, after_t, (after_t - query_t) * 1000, (query_t - before_t) * 1000, after_idx);
            return (std::abs(after_t - query_t) < MAX_SYNC_DELTA_SEC) ? after : nullptr;
        } else {
            // RCLCPP_INFO(
            //     get_logger(),
            //     "Matched scan time %.3f to before pose at %.3f (delta %.0f ms) [after delta %.0f ms] index %zu",
            //     query_t, before_t, (query_t - before_t) * 1000, (after_t - query_t) * 1000, before_idx);
            return (std::abs(query_t - before_t) < MAX_SYNC_DELTA_SEC) ? before : nullptr;
        }

        // If you want interpolation instead of nearest-pose selection,
        // that can be added here.
    }

    geometry_msgs::msg::PoseStamped::SharedPtr match_scan_to_pose(
        const sensor_msgs::msg::LaserScan::SharedPtr & scan)
    {
        double scan_centre_t = stamp_to_sec(scan->header.stamp) + scan->scan_time * 0.5;
        return get_pose_at_time_sec(scan_centre_t);
    }

    void mark_free(int gx, int gy)
    {
        if (!in_bounds(gx, gy)) {
            return;
        }
        int i = idx(gx, gy);
        log_[i] = std::max(static_cast<float>(log_[i] - L_FREE), static_cast<float>(min_log_));
    }

    void mark_occupied_thick(int gx, int gy)
    {
        int r = occ_radius_;
        int x0 = std::max(0, gx - r);
        int x1 = std::min(width_, gx + r + 1);
        int y0 = std::max(0, gy - r);
        int y1 = std::min(height_, gy + r + 1);

        for (int y = y0; y < y1; ++y) {
            for (int x = x0; x < x1; ++x) {
                int i = idx(x, y);
                log_[i] = std::clamp(
                    static_cast<float>(log_[i] + L_OCC),
                    static_cast<float>(min_log_),
                    static_cast<float>(max_log_));
            }
        }
    }

    void integrate_scan(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
    {
        double angle = scan->angle_min;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];

            if (r > scan->range_max || r < scan->range_min) {
                angle += scan->angle_increment;
                continue;
            }

            double ray_t = stamp_to_sec(scan->header.stamp) + static_cast<double>(i) * scan->time_increment;
            auto pose = get_pose_at_time_sec(ray_t);
            if (!pose) {
                angle += scan->angle_increment;
                continue;
            }

            double bx = pose->pose.position.x;
            double by = pose->pose.position.y;
            double yaw = yaw_from_quat(
                pose->pose.orientation.x,
                pose->pose.orientation.y,
                pose->pose.orientation.z,
                pose->pose.orientation.w);

            double tx = bx + std::cos(yaw) * laser_offset_x_ - std::sin(yaw) * laser_offset_y_;
            double ty = by + std::sin(yaw) * laser_offset_x_ + std::cos(yaw) * laser_offset_y_;
            double laser_yaw = yaw;

            auto [ogx, ogy] = world_to_grid(tx, ty);
            if (!in_bounds(ogx, ogy)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Robot outside occupancy grid.");
                angle += scan->angle_increment;
                continue;
            }

            double a = laser_yaw + angle;
            angle += scan->angle_increment;

            bool hit_obstacle = false;
            double ray_range = HARD_RANGE_MAX;

            if (std::isfinite(r)) {
                if (r < rmin_) {
                    continue;
                }
                if (r > HARD_RANGE_MAX) {
                    ray_range = HARD_RANGE_MAX;
                } else {
                    ray_range = r;
                    hit_obstacle = true;
                }
            } else {
                ray_range = HARD_RANGE_MAX;
            }

            double ex = tx + ray_range * std::cos(a);
            double ey = ty + ray_range * std::sin(a);

            auto [egx_raw, egy_raw] = world_to_grid(ex, ey);
            auto [egx, egy] = clamp_grid(egx_raw, egy_raw);

            auto cells = bresenham(ogx, ogy, egx, egy);
            if (cells.size() < 2) {
                continue;
            }

            if (hit_obstacle) {
                for (size_t k = 0; k + 1 < cells.size(); ++k) {
                    mark_free(cells[k].first, cells[k].second);
                }
                mark_occupied_thick(cells.back().first, cells.back().second);
            } else {
                for (const auto & cell : cells) {
                    mark_free(cell.first, cell.second);
                }
            }
        }
    }

    void process_and_publish()
    {
        if (pose_buffer_.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No Vicon poses received yet.");
            return;
        }

        int processed = 0;
        int dropped = 0;

        while (!scan_buffer_.empty()) {
            // RCLCPP_INFO(get_logger(), "Scan buffer size: %zu", scan_buffer_.size());
            auto scan = scan_buffer_.front();
            scan_buffer_.pop_front();

            auto matched_pose = match_scan_to_pose(scan);
            if (!matched_pose) {
                dropped++;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Dropped scan — no pose within %.0f ms (scan_t=%.3f)",
                    MAX_SYNC_DELTA_SEC * 1000.0,
                    stamp_to_sec(scan->header.stamp));
                continue;
            }

            integrate_scan(scan);
            processed++;
        }

        if (processed == 0 && dropped == 0) {
            return;
        }

        if (dropped > 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "This cycle: %d scans processed, %d dropped.",
                processed, dropped);
        }

        num_of_scans_processed_ += 1;

        if (num_of_scans_processed_ > 10) {
            publish_map();
        }
    }

    void publish_map()
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = map_frame_;
        msg.info.resolution = res_;
        msg.info.width = width_;
        msg.info.height = height_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        double log_range = max_log_ - min_log_;
        msg.data.resize(log_.size());

        for (size_t i = 0; i < log_.size(); ++i) {
            // if (log_[i] == 0.0f) {
            if (log_[i] <= 0.0f && log_[i] > -0.3f) {
                msg.data[i] = -1;
            } else {
                int8_t scaled = static_cast<int8_t>(((log_[i] - min_log_) / log_range) * 100.0);
                scaled = std::clamp<int>(scaled, 0, 100);

                if (scaled < 30) {
                    msg.data[i] = 0;
                } else {
                    msg.data[i] = 1;
                }
            }
        }

        map_pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}