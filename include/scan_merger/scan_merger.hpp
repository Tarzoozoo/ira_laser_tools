#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rclcpp/rclcpp.hpp"

//PCL to scan
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <limits>
#include <utility>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"

#include "laser_geometry/laser_geometry.hpp"

#include <Eigen/Dense> //from ira_lab

namespace scan_merger {
class LaserscanMergerNode : public rclcpp::Node {
 public:
  LaserscanMergerNode(const rclcpp::NodeOptions &options);
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
 private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<PointCloudMsg,
                                                      PointCloudMsg>;

  // main callback function
  void pointCloudCallback(const PointCloudMsg::ConstSharedPtr &msg1,
                          const PointCloudMsg::ConstSharedPtr &msg2);

  void concatenatePointCloud(const PointCloudMsg &pc_in, PointCloudMsg &pc_out,
                             uint32_t &concat_idx,
                             const Eigen::Affine3f &affine_tf);

  void convertToAffine3f(const geometry_msgs::msg::Transform &tf,
                         Eigen::Affine3f &af);

  void cloudCallback(sensor_msgs::msg::PointCloud2 fused_point_cloud_);


  Eigen::Affine3f affine3f_front_lidar_;
  Eigen::Affine3f affine3f_back_lidar_;

  // message filter synchronizer object
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>>
      point_cloud_synchronizer_;

  // final fused point cloud publisher object
  rclcpp::Publisher<PointCloudMsg>::SharedPtr fused_point_cloud_publisher_;
  
  //used to register final callback from fused_point_cloud to scan_msg

  // need two subscribers (front and back lidars)
  std::unique_ptr<message_filters::Subscriber<PointCloudMsg>>
      front_lidar_subscriber_;
  std::unique_ptr<message_filters::Subscriber<PointCloudMsg>>
      back_lidar_subscriber_;

  // final fused point cloud msg object
  PointCloudMsg fused_point_cloud_;

  std::string fused_frame_name_;
  uint32_t fused_point_cloud_max_capacity_;

  //PCL to scan

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
  int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
    range_max_;
  bool use_inf_;
  double inf_epsilon_;
  laser_geometry::LaserProjection projector_;

};
}  // namespace scan_merger