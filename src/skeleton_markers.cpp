#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2/exceptions.h"
#include <chrono>
#include <vector>

using namespace rclcpp;

class SkeletonMarkers : public Node
{
    Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                      tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>           tf_listener_{nullptr};
    TimerBase::SharedPtr                                  timer_{nullptr};
    std::string                                           fixed_frame_;
    std::vector<std::string>                              skeleton_frames_;
    visualization_msgs::msg::Marker                       marker_;

public:

    SkeletonMarkers()
    : Node("skeleton_markers")
    {
        fixed_frame_ = this->declare_parameter(
            "fixed_frame", "camera_depth_frame");

        skeleton_frames_ = {
            "head",
            "neck",
            "torso",
            "left_shoulder",
            "left_elbow",
            "left_hand",
            "left_hip",
            "left_knee",
            "left_foot",
            "right_shoulder",
            "right_elbow",
            "right_hand",
            "right_hip",
            "right_knee",
            "right_foot"};

        skeleton_frames_ = this->declare_parameter(
            "skeleton_frames", skeleton_frames_);

        tf_buffer_ = 
            std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = 
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                "skeleton_markers", 1);

        initialize_marker();

        // Make sure we see the camera_depth_frame
        std::string msg;
        if (!tf_buffer_->canTransform(fixed_frame_, fixed_frame_, 
            tf2::TimePointZero, tf2::Duration(30), &msg))
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Timeout during waiting for frame! %s", msg.c_str());
            rclcpp::shutdown();
            return;
        }

        auto rate = this->declare_parameter("rate", 30);
        auto duration = std::chrono::milliseconds(1000/rate);
        timer_ = this->create_wall_timer(
            duration, std::bind(&SkeletonMarkers::on_timer, this));
    }

private:

    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;
        marker_.header.stamp = this->get_clock()->now();
        marker_.points.clear();

        for (auto & head_frame : tf_buffer_->getAllFrameNames())
        {
            if (head_frame.rfind("head_", 0) == 0)
            {
                auto user_index = head_frame;
                user_index.replace(0, 5, ""); 
                RCLCPP_DEBUG(this->get_logger(),"%s %s", head_frame.c_str(), user_index.c_str());

                try 
                {
                    t = tf_buffer_->lookupTransform(
                        fixed_frame_, head_frame,
                        tf2::TimePointZero);
                    RCLCPP_DEBUG(this->get_logger(),"skeleton_detected");

                    for (auto& frame : skeleton_frames_)
                    {
                        // Append the user_index to the frame name
                        auto skel_frame = frame + "_" + user_index;
                        t = tf_buffer_->lookupTransform(
                            fixed_frame_, skel_frame, tf2::TimePointZero);
                        // We only need the origin of each skeleton frame
                        // relative to the fixed frame
                        geometry_msgs::msg::Point position;
                        position.x = t.transform.translation.x;
                        position.y = t.transform.translation.y;
                        position.z = t.transform.translation.z;
                        marker_.points.push_back(position);
                    }
                    publisher_->publish(marker_);
                } 
                catch (const tf2::TransformException & ex)
                {
                    RCLCPP_WARN(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        fixed_frame_.c_str(), head_frame.c_str(), ex.what());
                    return;
                }
            }
        }
    }

    void initialize_marker()
    {
        std::vector<double> color{0.0, 1.0, 0.0, 1.0};
        color         = this->declare_parameter("color", color);
        auto scale    = this->declare_parameter("scale", 0.07);
        auto lifetime = this->declare_parameter("lifetime", 0);
        auto id       = this->declare_parameter("id", 0);
        auto ns       = this->declare_parameter("ns", "skeleton_markers");
        
        marker_.header.frame_id = fixed_frame_;
        marker_.ns = ns;
        marker_.id = id;
        marker_.type = visualization_msgs::msg::Marker::POINTS;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.lifetime = rclcpp::Duration(lifetime, 0);
        marker_.scale.x = scale;
        marker_.scale.y = scale;
        marker_.color.r = color[0];
        marker_.color.g = color[1];
        marker_.color.b = color[2];
        marker_.color.a = color[3];
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SkeletonMarkers>());
    rclcpp::shutdown();
    return 0;
}
