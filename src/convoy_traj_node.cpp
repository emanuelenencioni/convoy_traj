#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class ConvoyTrajNode : public rclcpp::Node
{
public:
  ConvoyTrajNode()
  : Node("convoy_traj_node")
  {
    RCLCPP_INFO(this->get_logger(), "Convoy Trajectory Node has been started.");

    tag_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/tag_pose", 10);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ConvoyTrajNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped_0, transform_stamped_1;
    try {
      // Assuming the base frame is 'camera' and the tag is 'tag_0'
      // You might need to change these frame names
      // Added a timeout to wait for the transform to be available
      transform_stamped_0 = tf_buffer_->lookupTransform("zed_left_camera_optical_frame", "tag36h11:0", tf2::TimePoint(), std::chrono::milliseconds(50));
      transform_stamped_1 = tf_buffer_->lookupTransform("zed_left_camera_optical_frame", "tag36h11:1", tf2::TimePoint(), std::chrono::milliseconds(50));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    // Compute average of the two poses
    geometry_msgs::msg::TransformStamped averaged_transform;
    averaged_transform.header = transform_stamped_0.header;
    
    // Average translation
    averaged_transform.transform.translation.x = (transform_stamped_0.transform.translation.x + transform_stamped_1.transform.translation.x) / 2.0;
    averaged_transform.transform.translation.y = (transform_stamped_0.transform.translation.y + transform_stamped_1.transform.translation.y) / 2.0;
    averaged_transform.transform.translation.z = (transform_stamped_0.transform.translation.z + transform_stamped_1.transform.translation.z) / 2.0;
    
    // Average rotation using quaternion SLERP
    tf2::Quaternion q0, q1;
    tf2::fromMsg(transform_stamped_0.transform.rotation, q0);
    tf2::fromMsg(transform_stamped_1.transform.rotation, q1);
    
    // Use SLERP with 0.5 weight for proper quaternion averaging
    tf2::Quaternion q_avg = q0.slerp(q1, 0.5);
    averaged_transform.transform.rotation.x = q_avg.x();
    averaged_transform.transform.rotation.y = q_avg.y();
    averaged_transform.transform.rotation.z = q_avg.z();
    averaged_transform.transform.rotation.w = q_avg.w();

    if (first_transform_) {
      filtered_transform_ = averaged_transform;
      first_transform_ = false;
    } else {
      // Apply exponential moving average filter on the averaged transform
      // Filter translation
      filtered_transform_.transform.translation.x = alpha_ * averaged_transform.transform.translation.x + (1.0 - alpha_) * filtered_transform_.transform.translation.x;
      filtered_transform_.transform.translation.y = alpha_ * averaged_transform.transform.translation.y + (1.0 - alpha_) * filtered_transform_.transform.translation.y;
      filtered_transform_.transform.translation.z = alpha_ * averaged_transform.transform.translation.z + (1.0 - alpha_) * filtered_transform_.transform.translation.z;

      // Filter rotation using SLERP (Spherical Linear Interpolation)
      tf2::Quaternion q_new, q_old;
      tf2::fromMsg(averaged_transform.transform.rotation, q_new);
      tf2::fromMsg(filtered_transform_.transform.rotation, q_old);
      
      tf2::Quaternion q_filtered = q_old.slerp(q_new, alpha_);
      filtered_transform_.transform.rotation.x = q_filtered.x();
      filtered_transform_.transform.rotation.y = q_filtered.y();
      filtered_transform_.transform.rotation.z = q_filtered.z();
      filtered_transform_.transform.rotation.w = q_filtered.w();
    }


    geometry_msgs::msg::PoseWithCovarianceStamped pose_to_publish;
    pose_to_publish.header.stamp = this->get_clock()->now();
    pose_to_publish.header.frame_id = "front_vehicle";
    pose_to_publish.pose.pose.position.x = filtered_transform_.transform.translation.x;
    pose_to_publish.pose.pose.position.y = filtered_transform_.transform.translation.y;
    pose_to_publish.pose.pose.position.z = filtered_transform_.transform.translation.z;
    pose_to_publish.pose.pose.orientation = filtered_transform_.transform.rotation;

    // Add covariance to the pose. These are example values and should be tuned.
    pose_to_publish.pose.covariance[0] = 0.1; // x
    pose_to_publish.pose.covariance[7] = 0.1; // y
    pose_to_publish.pose.covariance[14] = 0.1; // z
    pose_to_publish.pose.covariance[21] = 0.05; // roll
    pose_to_publish.pose.covariance[28] = 0.05; // pitch
    pose_to_publish.pose.covariance[35] = 0.05; // yaw

    tag_pose_pub_->publish(pose_to_publish);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  
  // Variables for filtering
  geometry_msgs::msg::TransformStamped filtered_transform_;
  bool first_transform_{true};
  double alpha_{0.1}; // Smoothing factor (0.0 < alpha <= 1.0). Smaller values mean more smoothing.
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConvoyTrajNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
