#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class OmoController : public rclcpp::Node
{
public:
    OmoController() : Node("Ref_path_omo")
    {
        // Subscriber for odom (odometry)
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "omo0/odom", 10, std::bind(&OmoController::odomCallback, this, std::placeholders::_1));

        // Publisher for path (to visualize in rviz2)
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        ref_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("ref_path", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("omo0/cmd_vel", 10);
    }

    void moveTowardsGoal(const std::vector<geometry_msgs::msg::PoseStamped>& goal_poses)
    {
        rclcpp::Rate rate(10); // 10 Hz control rate
        size_t current_index = 0;

        while (rclcpp::ok() && current_index < goal_poses.size()) {
            // Get the current goal pose
            const auto& goal_pose = goal_poses[current_index];

            // Calculate twist message to move towards goal
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.2; // Constant linear velocity
            twist.angular.z = 0.0; // No angular velocity

            // Publish the twist message
            cmd_vel_publisher_->publish(twist);

            // Calculate distance to goal
            double distance = sqrt(pow(goal_pose.pose.position.x - current_pose_.position.x, 2) +
                                   pow(goal_pose.pose.position.y - current_pose_.position.y, 2));

            // Check if goal reached
            if (distance < 0.1) {
                RCLCPP_INFO(this->get_logger(), "Goal %zu reached!", current_index + 1);
                ++current_index; // Move to the next goal
            }

            // Add future path to ref_path
            nav_msgs::msg::Path ref_path;
            ref_path.header.frame_id = "map";
            ref_path.header.stamp = rclcpp::Clock().now();

            for (double x = current_pose_.position.x; x <= current_pose_.position.x + 2.0; x += 0.1) {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = x;
                pose_stamped.pose.position.y = 0.0;
                pose_stamped.pose.position.z = 0.0;
                pose_stamped.pose.orientation.x = 0.0;
                pose_stamped.pose.orientation.y = 0.0;
                pose_stamped.pose.orientation.z = 0.0;
                pose_stamped.pose.orientation.w = 1.0;
                pose_stamped.header.frame_id = "map";
                ref_path.poses.emplace_back(pose_stamped);
            }

            // Publish ref_path
            ref_path_publisher_->publish(ref_path);

            // Sleep to control rate
            rate.sleep();
            rclcpp::spin_some(this->shared_from_this());
        }
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extracting pose from odometry message
        current_pose_ = msg->pose.pose;

        // Adding pose to the path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = current_pose_;
        pose_stamped.header = msg->header;
        path_.poses.emplace_back(pose_stamped);

        // Publishing the path
        path_.header.frame_id = "map";
        path_publisher_->publish(path_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmoController>();

    // Create goal poses
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
    for (double x = 0.0; x <= 10.0; x += 1.0) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.frame_id = "map";
        goal_poses.emplace_back(pose);
    }

    node->moveTowardsGoal(goal_poses);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
