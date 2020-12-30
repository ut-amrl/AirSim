#include <glog/logging.h>
#include <ros/spinner.h>
#include "airsim_ros_wrapper.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk

  ros::init(argc, argv, "airsim_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string host_ip = "localhost";
  std::string vehicle_type = "car";
  nh_private.getParam("host_ip", host_ip);
  nh_private.getParam("vehicle_type", vehicle_type);
  AirsimROSWrapper airsim_ros_wrapper(nh, nh_private, host_ip, vehicle_type);

  if (airsim_ros_wrapper.is_used_img_timer_cb_queue_) {
    airsim_ros_wrapper.img_async_spinner_.start();
  }

  if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_) {
    airsim_ros_wrapper.lidar_async_spinner_.start();
  }

  ros::spin();

  return 0;
}
