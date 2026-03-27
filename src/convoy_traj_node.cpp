#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

namespace convoy_traj
{

class ConvoyTrajNode : public rclcpp::Node
{
public:
  explicit ConvoyTrajNode(const rclcpp::NodeOptions & options)
  : Node("convoy_traj_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Convoy Trajectory Node has been started.");

    // --- Frame name parameters ---
    this->declare_parameter<std::string>("tag0_frame_id", "tag36h11:0");
    this->declare_parameter<std::string>("tag1_frame_id", "tag36h11:1");
    this->declare_parameter<std::string>("camera_frame_id", "zed_left_camera_optical_frame");

    // --- Front-axes offset parameters ---
    this->declare_parameter<double>("axes_offset_x", 0.0);
    this->declare_parameter<double>("axes_offset_y", 0.0);
    this->declare_parameter<double>("axes_offset_z", 0.0);
    this->declare_parameter<double>("alpha", 0.1);

    // Maximum age of a TF transform before the tag is considered not visible.
    // Must be larger than the AprilTag detector's pipeline latency (~100–500 ms
    // typical) but small enough to catch a tag that has actually disappeared.
    // Default: 1000 ms — increase if your detector runs slower.
    this->declare_parameter<int>("tf_max_age_ms", 5000);

    tag0_frame_id_   = this->get_parameter("tag0_frame_id").as_string();
    tag1_frame_id_   = this->get_parameter("tag1_frame_id").as_string();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    axes_offset_x_   = this->get_parameter("axes_offset_x").as_double();
    axes_offset_y_   = this->get_parameter("axes_offset_y").as_double();
    axes_offset_z_   = this->get_parameter("axes_offset_z").as_double();
    alpha_           = this->get_parameter("alpha").as_double();
    tf_max_age_s_    = this->get_parameter("tf_max_age_ms").as_int() / 1000.0;

    RCLCPP_INFO(this->get_logger(), "Tag frames: %s, %s",
                tag0_frame_id_.c_str(), tag1_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera frame: %s", camera_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Front axes offset: x=%.3f, y=%.3f, z=%.3f",
                axes_offset_x_, axes_offset_y_, axes_offset_z_);
    RCLCPP_INFO(this->get_logger(), "Filter alpha: %.3f", alpha_);
    RCLCPP_INFO(this->get_logger(), "TF max age: %.3f s", tf_max_age_s_);

    // --- Inter-tag calibration parameters ---
    // Loaded from tag_calibration.yaml (written by calibrate_tags.py).
    // Encodes the fixed transform tag0 -> tag1 and enables single-tag fallback:
    // when one tag is occluded, the other's pose is reconstructed via this transform.
    this->declare_parameter<bool>("tag_calib_enabled", false);
    this->declare_parameter<double>("tag_calib_tx", 0.0);
    this->declare_parameter<double>("tag_calib_ty", 0.0);
    this->declare_parameter<double>("tag_calib_tz", 0.0);
    this->declare_parameter<double>("tag_calib_qx", 0.0);
    this->declare_parameter<double>("tag_calib_qy", 0.0);
    this->declare_parameter<double>("tag_calib_qz", 0.0);
    this->declare_parameter<double>("tag_calib_qw", 1.0);
    this->declare_parameter<int>("tag_calib_samples", 0);

    tag_calib_enabled_ = this->get_parameter("tag_calib_enabled").as_bool();

    if (tag_calib_enabled_) {
      tf2::Vector3 t(
        this->get_parameter("tag_calib_tx").as_double(),
        this->get_parameter("tag_calib_ty").as_double(),
        this->get_parameter("tag_calib_tz").as_double());
      tf2::Quaternion q(
        this->get_parameter("tag_calib_qx").as_double(),
        this->get_parameter("tag_calib_qy").as_double(),
        this->get_parameter("tag_calib_qz").as_double(),
        this->get_parameter("tag_calib_qw").as_double());
      q.normalize();
      calib_T_tag0_tag1_.setOrigin(t);
      calib_T_tag0_tag1_.setRotation(q);

      int samples = this->get_parameter("tag_calib_samples").as_int();
      RCLCPP_INFO(this->get_logger(),
        "Inter-tag calibration ENABLED (samples=%d). "
        "T_tag0_tag1: t=(%.4f, %.4f, %.4f) q=(%.4f, %.4f, %.4f, %.4f)",
        samples, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Inter-tag calibration DISABLED. Run calibrate_tags.py to enable "
        "single-tag fallback.");
    }

    // --- Publishers ---
    tag_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/tag_pose", 10);
    axes_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/front_axes_pose", 10);

    // --- TF infrastructure ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publish_static_transform();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ConvoyTrajNode::timer_callback, this));
  }

private:
  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------

  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->get_clock()->now();
    static_transform.header.frame_id = "front_vehicle";
    static_transform.child_frame_id = "front_axes";
    static_transform.transform.translation.x = axes_offset_x_;
    static_transform.transform.translation.y = axes_offset_y_;
    static_transform.transform.translation.z = axes_offset_z_;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(static_transform);
    RCLCPP_INFO(this->get_logger(), "Published static transform: front_vehicle -> front_axes");
  }

  /// Convert a TransformStamped's translation+rotation into a tf2::Transform.
  static tf2::Transform to_tf2(const geometry_msgs::msg::TransformStamped & ts)
  {
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(
      ts.transform.translation.x,
      ts.transform.translation.y,
      ts.transform.translation.z));
    tf2::Quaternion q(
      ts.transform.rotation.x,
      ts.transform.rotation.y,
      ts.transform.rotation.z,
      ts.transform.rotation.w);
    T.setRotation(q);
    return T;
  }

  /// Fill translation/rotation fields of a TransformStamped from a tf2::Transform.
  static void from_tf2(const tf2::Transform & T,
                       geometry_msgs::msg::TransformStamped & ts)
  {
    const tf2::Vector3 & o = T.getOrigin();
    ts.transform.translation.x = o.x();
    ts.transform.translation.y = o.y();
    ts.transform.translation.z = o.z();
    ts.transform.rotation = tf2::toMsg(T.getRotation());
  }

  /// Returns true if the transform's stamp is within tf_max_age_s_ of now.
  bool is_fresh(const geometry_msgs::msg::TransformStamped & ts)
  {
    rclcpp::Time stamp(ts.header.stamp, this->get_clock()->get_clock_type());
    double age = (this->get_clock()->now() - stamp).seconds();
    return age <= tf_max_age_s_;
  }

  // --------------------------------------------------------------------------
  // Timer callback
  // --------------------------------------------------------------------------

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped ts0, ts1;
    bool ok0 = false;
    bool ok1 = false;

    try {
      ts0 = tf_buffer_->lookupTransform(
        camera_frame_id_, tag0_frame_id_,
        tf2::TimePoint(), std::chrono::milliseconds(50));
      ok0 = is_fresh(ts0);
      if (!ok0) {
        RCLCPP_WARN(this->get_logger(),
          "tag0 transform is stale (%.2f s old — tag likely disappeared)",
          (this->get_clock()->now() -
           rclcpp::Time(ts0.header.stamp, this->get_clock()->get_clock_type())).seconds());
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get tag0 transform: %s", ex.what());
    }

    try {
      ts1 = tf_buffer_->lookupTransform(
        camera_frame_id_, tag1_frame_id_,
        tf2::TimePoint(), std::chrono::milliseconds(50));
      ok1 = is_fresh(ts1);
      if (!ok1) {
        RCLCPP_WARN(this->get_logger(),
          "tag1 transform is stale (%.2f s old — tag likely disappeared)",
          (this->get_clock()->now() -
           rclcpp::Time(ts1.header.stamp, this->get_clock()->get_clock_type())).seconds());
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get tag1 transform: %s", ex.what());
    }

    // --- Single-tag fallback via calibrated inter-tag transform ---
    if (!ok0 && !ok1) {
      return;
    }

    if (!ok0 || !ok1) {
      if (!tag_calib_enabled_) {
        RCLCPP_WARN(this->get_logger(),
          "Only one tag visible and calibration is DISABLED — cannot reconstruct "
          "the missing tag. Run calibrate_tags.py to enable fallback.");
        return;
      }

      if (ok0 && !ok1) {
        // Reconstruct tag1: T_cam_tag1 = T_cam_tag0 * T_tag0_tag1_calib
        tf2::Transform T_cam_tag1 = to_tf2(ts0) * calib_T_tag0_tag1_;
        ts1.header = ts0.header;
        ts1.child_frame_id = tag1_frame_id_;
        from_tf2(T_cam_tag1, ts1);
        RCLCPP_WARN(this->get_logger(),
          "tag1 not visible — reconstructed from tag0 + calibration.");
      } else {
        // Reconstruct tag0: T_cam_tag0 = T_cam_tag1 * T_tag0_tag1_calib^-1
        tf2::Transform T_cam_tag0 = to_tf2(ts1) * calib_T_tag0_tag1_.inverse();
        ts0.header = ts1.header;
        ts0.child_frame_id = tag0_frame_id_;
        from_tf2(T_cam_tag0, ts0);
        RCLCPP_WARN(this->get_logger(),
          "tag0 not visible — reconstructed from tag1 + calibration.");
      }
    }

    // --- Average the two poses (both real or one reconstructed) ---
    geometry_msgs::msg::TransformStamped averaged_transform;
    averaged_transform.header = ts0.header;

    averaged_transform.transform.translation.x =
      (ts0.transform.translation.x + ts1.transform.translation.x) / 2.0;
    averaged_transform.transform.translation.y =
      (ts0.transform.translation.y + ts1.transform.translation.y) / 2.0;
    averaged_transform.transform.translation.z =
      (ts0.transform.translation.z + ts1.transform.translation.z) / 2.0;

    tf2::Quaternion q0, q1;
    tf2::fromMsg(ts0.transform.rotation, q0);
    tf2::fromMsg(ts1.transform.rotation, q1);
    tf2::Quaternion q_avg = q0.slerp(q1, 0.5);
    averaged_transform.transform.rotation.x = q_avg.x();
    averaged_transform.transform.rotation.y = q_avg.y();
    averaged_transform.transform.rotation.z = q_avg.z();
    averaged_transform.transform.rotation.w = q_avg.w();

    // --- Exponential moving average filter ---
    if (first_transform_) {
      filtered_transform_ = averaged_transform;
      first_transform_ = false;
    } else {
      filtered_transform_.transform.translation.x =
        alpha_ * averaged_transform.transform.translation.x +
        (1.0 - alpha_) * filtered_transform_.transform.translation.x;
      filtered_transform_.transform.translation.y =
        alpha_ * averaged_transform.transform.translation.y +
        (1.0 - alpha_) * filtered_transform_.transform.translation.y;
      filtered_transform_.transform.translation.z =
        alpha_ * averaged_transform.transform.translation.z +
        (1.0 - alpha_) * filtered_transform_.transform.translation.z;

      tf2::Quaternion q_new, q_old;
      tf2::fromMsg(averaged_transform.transform.rotation, q_new);
      tf2::fromMsg(filtered_transform_.transform.rotation, q_old);
      tf2::Quaternion q_filtered = q_old.slerp(q_new, alpha_);
      filtered_transform_.transform.rotation.x = q_filtered.x();
      filtered_transform_.transform.rotation.y = q_filtered.y();
      filtered_transform_.transform.rotation.z = q_filtered.z();
      filtered_transform_.transform.rotation.w = q_filtered.w();
    }

    // --- Apply rotation correction and publish /tag_pose ---
    geometry_msgs::msg::PoseWithCovarianceStamped pose_to_publish;
    pose_to_publish.header.stamp = this->get_clock()->now();
    pose_to_publish.header.frame_id = camera_frame_id_;
    pose_to_publish.pose.pose.position.x = filtered_transform_.transform.translation.x;
    pose_to_publish.pose.pose.position.y = filtered_transform_.transform.translation.y;
    pose_to_publish.pose.pose.position.z = filtered_transform_.transform.translation.z;

    // 180° roll + 180° yaw: aligns AprilTag optical convention with vehicle forward axis.
    tf2::Quaternion q_rotation;
    q_rotation.setRPY(M_PI, 0, M_PI);
    tf2::Quaternion q_filtered_rot;
    tf2::fromMsg(filtered_transform_.transform.rotation, q_filtered_rot);
    tf2::Quaternion q_corrected = q_filtered_rot * q_rotation;
    q_corrected.normalize();
    pose_to_publish.pose.pose.orientation = tf2::toMsg(q_corrected);

    pose_to_publish.pose.covariance[0]  = 0.1;   // x
    pose_to_publish.pose.covariance[7]  = 0.1;   // y
    pose_to_publish.pose.covariance[14] = 0.1;   // z
    pose_to_publish.pose.covariance[21] = 0.05;  // roll
    pose_to_publish.pose.covariance[28] = 0.05;  // pitch
    pose_to_publish.pose.covariance[35] = 0.05;  // yaw

    tag_pose_pub_->publish(pose_to_publish);

    // --- Broadcast dynamic TF: camera -> front_vehicle ---
    geometry_msgs::msg::TransformStamped front_vehicle_transform;
    front_vehicle_transform.header.stamp = this->get_clock()->now();
    front_vehicle_transform.header.frame_id = camera_frame_id_;
    front_vehicle_transform.child_frame_id = "front_vehicle";
    front_vehicle_transform.transform.translation.x = filtered_transform_.transform.translation.x;
    front_vehicle_transform.transform.translation.y = filtered_transform_.transform.translation.y;
    front_vehicle_transform.transform.translation.z = filtered_transform_.transform.translation.z;
    front_vehicle_transform.transform.rotation = tf2::toMsg(q_corrected);
    tf_broadcaster_->sendTransform(front_vehicle_transform);

    // --- Publish /front_axes_pose ---
    try {
      geometry_msgs::msg::TransformStamped axes_transform =
        tf_buffer_->lookupTransform(
          camera_frame_id_, "front_axes",
          tf2::TimePoint(), std::chrono::milliseconds(50));

      geometry_msgs::msg::PoseWithCovarianceStamped axes_pose;
      axes_pose.header.stamp = this->get_clock()->now();
      axes_pose.header.frame_id = camera_frame_id_;
      axes_pose.pose.pose.position.x = axes_transform.transform.translation.x;
      axes_pose.pose.pose.position.y = axes_transform.transform.translation.y;
      axes_pose.pose.pose.position.z = axes_transform.transform.translation.z;
      axes_pose.pose.pose.orientation = axes_transform.transform.rotation;
      axes_pose.pose.covariance = pose_to_publish.pose.covariance;
      axes_pose_pub_->publish(axes_pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get front_axes transform: %s", ex.what());
    }
  }

  // --------------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------------

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr axes_pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // EMA filter state
  geometry_msgs::msg::TransformStamped filtered_transform_;
  bool first_transform_{true};
  double alpha_{0.1};  // Smoothing factor (0 < alpha <= 1). Smaller = more smoothing.

  // Frame names (configurable via parameters)
  std::string tag0_frame_id_;
  std::string tag1_frame_id_;
  std::string camera_frame_id_;

  // Front axes offset
  double axes_offset_x_{0.0};
  double axes_offset_y_{0.0};
  double axes_offset_z_{0.0};

  // Inter-tag calibration
  bool tag_calib_enabled_{false};
  tf2::Transform calib_T_tag0_tag1_;  // Fixed transform: tag0 frame -> tag1 frame

  // TF freshness threshold: transforms older than this are treated as "tag gone"
  double tf_max_age_s_{1.0};
};

}  // namespace convoy_traj

RCLCPP_COMPONENTS_REGISTER_NODE(convoy_traj::ConvoyTrajNode)
