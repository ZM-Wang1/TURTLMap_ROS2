#pragma once

#include <rclcpp/rclcpp.hpp>
#include "TURTLMap/Posegraph.h"
#include <string>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <turtlmap/srv/save_trajectory.hpp>
#include <dvl_msgs/msg/dvl.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <condition_variable>
#include <memory>

namespace pose_graph_backend
{
  class PosegraphBackendOnline : public rclcpp::Node
  {
  private:
    // --- ROS node handle ---
    // rclcpp::Node::SharedPtr this;

    // --- Pose‑graph backend ---
    AUVPoseGraph *posegraph_{};

    // --- Subscribers ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_local_subscriber_;

    // --- Service ---
    rclcpp::Service<turtlmap::srv::SaveTrajectory>::SharedPtr save_traj_service_;
    void save_trajctory_(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<turtlmap::srv::SaveTrajectory::Request> req,
        std::shared_ptr<turtlmap::srv::SaveTrajectory::Response> res);

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dvl_local_pose_publisher_;

    // --- TF broadcaster ---
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Threading helpers ---
    std::mutex mtx_;
    std::condition_variable cv_;

    // --- Internal states and buffers (unchanged) ---
    std::vector<double> kf_timestamps_;
    std::vector<double> traj_timestamps_;
    std::vector<gtsam::Pose3> traj_poses_;

    std::int64_t frame_count_{};
    double first_depth_{0.0};
    double prev_dvl_time_{0.0};
    double prev_dvl_local_time_{0.0};
    double current_kf_time_{0.0};
    double prev_kf_time_{0.0};
    bool new_kf_flag_{false};
    gtsam::Pose3 prev_dvl_local_pose_;
    gtsam::Pose3 T_w_wd_;
    double kf_gap_time_{};

    bool is_using_dvl_v2_factor{true};
    bool is_rot_initialized_{false};
    gtsam::Rot3 imu_latest_rot_;
    gtsam::Rot3 dvl_prev_rot_;
    int imu_init_count_{0};
    std::vector<gtsam::Vector3> imu_init_acc_;
    std::vector<gtsam::Vector3> imu_init_rot_;
    gtsam::Pose3 latest_kf_pose_;
    gtsam::Pose3 latest_publish_pose_;
    gtsam::Vector3 first_dvl_vel_;
    double first_kf_time_{0.0};
    gtsam::PreintegratedCombinedMeasurements *pim_dvl_{};
    gtsam::Vector3 latest_dvl_vel_;
    gtsam::Pose3 latest_dvl_pose_;
    int imu_count_{0};
    gtsam::NavState latest_imu_prop_state_;

    void callbackIMU(sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void callbackDVL(dvl_msgs::msg::DVL::SharedPtr dvl_msg);
    void callbackBaro(sensor_msgs::msg::FluidPressure::SharedPtr baro_msg);
    void callbackDVLLocal(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr dvl_local_msg);

    // --- Worker threads ---
    void mainLoop();
    void kfLoop();

  public:
    explicit PosegraphBackendOnline(const std::string &config_file);
    ~PosegraphBackendOnline();
  };
}
