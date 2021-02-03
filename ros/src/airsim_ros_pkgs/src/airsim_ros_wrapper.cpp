#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(AirsimROSWrapper, nodelet::Nodelet)
#include "common/AirSimSettings.hpp"

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string>
    AirsimROSWrapper::image_type_int_to_string_map_ = {
        {0, "Scene"},
        {1, "DepthPlanner"},
        {2, "DepthPerspective"},
        {3, "DepthVis"},
        {4, "DisparityNormalized"},
        {5, "Segmentation"},
        {6, "SurfaceNormals"},
        {7, "Infrared"}};

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private,
                                   const std::string& host_ip,
                                   const std::string& vehicle_type)
    : nh_(nh),
      nh_private_(nh_private),
      img_async_spinner_(
          1, &img_timer_cb_queue_),  // a thread for image callbacks to be
                                     // 'spun' by img_async_spinner_
      lidar_async_spinner_(
          1, &lidar_timer_cb_queue_),  // same as above, but for lidar
      airsim_client_(host_ip),
      airsim_client_images_(host_ip),
      airsim_client_lidar_(host_ip),
      airsim_car_client_(host_ip),
      airsim_car_client_images_(host_ip),
      airsim_car_client_lidar_(host_ip) {
  if (vehicle_type == "car") {
    vehicle_type_ = CAR;
    std::cout << "vehicle type: car" << std::endl;
  } else if (vehicle_type == "multirotor") {
    vehicle_type_ = MULTIROTOR;
    std::cout << "vehicle type: multirotor" << std::endl;
  } else {
    LOG(FATAL) << "Unknown vehicle type " << vehicle_type;
  }

  is_used_lidar_timer_cb_queue_ = false;
  is_used_img_timer_cb_queue_ = false;

  // Set the constant transformation between NWU and NED
  tf2::Quaternion quat_tf;
  tf2::Vector3 vec_tf(0, 0, 0);
  quat_tf.setRPY(M_PI, 0.0, 0.0);
  geometry_msgs::Quaternion quat_msg;
  geometry_msgs::Vector3 vec_msg;
  tf2::convert(quat_tf, quat_msg);
  tf2::convert(vec_tf, vec_msg);

  trans_ned_nwu_.transform.translation = vec_msg;
  trans_ned_nwu_.transform.rotation = quat_msg;
  trans_ned_nwu_.child_frame_id = "nwu";
  trans_ned_nwu_.header.frame_id = "ned";

  trans_nwu_ned_ = trans_ned_nwu_;
  trans_nwu_ned_.child_frame_id = "ned";
  trans_nwu_ned_.header.frame_id = "nwu";

  if (use_nwu_std_) {
    frame_name_base_link_ = "/base_link";
    world_frame_id_ = "map";
  } else {
    frame_name_base_link_ = "/odom_local_ned";
    world_frame_id_ = "world_ned";  // todo rosparam?
  }

  initialize_ros();

  std::cout << "AirsimROSWrapper Initialized!\n";
  // intitialize placeholder control commands
  // vel_cmd_ = VelCmd();
  // gimbal_cmd_ = GimbalCmd();
}

void AirsimROSWrapper::initialize_airsim() {
  // todo do not reset if already in air?
  try {
    switch (vehicle_type_) {
      case CAR:
        airsim_car_client_.confirmConnection();
        airsim_car_client_images_.confirmConnection();
        airsim_car_client_lidar_.confirmConnection();

        if (use_api_control_) {
          for (const auto& vehicle_name : vehicle_names_) {
            airsim_car_client_.enableApiControl(true, vehicle_name);
            airsim_car_client_.enableApiControl(true, vehicle_name);
            airsim_car_client_.enableApiControl(true, vehicle_name);
            // todo exposes as rosservice?
            airsim_car_client_.armDisarm(true, vehicle_name);
            airsim_car_client_.armDisarm(true, vehicle_name);
            airsim_car_client_.armDisarm(true, vehicle_name);
          }
        }

        origin_geo_point_ = airsim_car_client_.getHomeGeoPoint("");
        break;
      case MULTIROTOR:
        airsim_client_.confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();

        if (use_api_control_) {
          for (const auto& vehicle_name : vehicle_names_) {
            airsim_client_.enableApiControl(true, vehicle_name);
            airsim_client_.enableApiControl(true, vehicle_name);
            airsim_client_.enableApiControl(true, vehicle_name);
            // todo exposes as rosservice?
            airsim_client_.armDisarm(true, vehicle_name);
            airsim_client_.armDisarm(true, vehicle_name);
            airsim_client_.armDisarm(true, vehicle_name);
          }
        }

        origin_geo_point_ = airsim_client_.getHomeGeoPoint("");
        // todo there's only one global origin geopoint for environment.
        // but airsim API accept a parameter vehicle_name? inside
        // carsimpawnapi.cpp, there's   a   geopoint being assigned in the
        // constructor. by?
        break;
      default:
        LOG(FATAL) << "Unknown vehicle type";
    }

    origin_geo_point_msg_ =
        get_gps_msg_from_airsim_geo_point(origin_geo_point_);
  } catch (rpc::rpc_error& e) {
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API, something went wrong."
              << std::endl
              << msg << std::endl;
  }
}

void AirsimROSWrapper::initialize_ros() {
  // ros params
  nh_private_.getParam("is_vulkan", is_vulkan_);
  nh_private_.getParam("update_airsim_control_every_n_sec",
                       update_airsim_timestep_);
  nh_private_.getParam("use_api_control", use_api_control_);
  vel_cmd_duration_ = 0.05;  // todo rosparam
  // todo enforce dynamics constraints in this node as well?
  // nh_.getParam("max_vert_vel_", max_vert_vel_);
  // nh_.getParam("max_horz_vel", max_horz_vel_)

  create_ros_pubs_from_settings_json();

  switch (vehicle_type_) {
    case CAR:
      airsim_control_update_timer_ =
          nh_private_.createTimer(ros::Duration(update_airsim_timestep_),
                                  &AirsimROSWrapper::car_state_timer_cb,
                                  this);
      break;
    case MULTIROTOR:
      airsim_control_update_timer_ =
          nh_private_.createTimer(ros::Duration(update_airsim_timestep_),
                                  &AirsimROSWrapper::drone_state_timer_cb,
                                  this);
      break;
    default:
      LOG(FATAL) << "Unknown vehicle type";
  }
}

void AirsimROSWrapper::add_ros_car(const std::string& vehicle_name) {
  CarROS car_ros(nh_private_);
  car_ros.odom_frame_id = vehicle_name + frame_name_base_link_;
  car_ros.vehicle_name = vehicle_name;
  car_ros.odom_local_ned_pub =
      nh_private_.advertise<nav_msgs::Odometry>(vehicle_name + "/odom", 10);
  car_ros.collision_info_pub =
      nh_private_.advertise<airsim_ros_pkgs::CollisionInfo>(
          vehicle_name + "/collision_info", 10);
  car_ros.global_gps_pub = nh_private_.advertise<sensor_msgs::NavSatFix>(
      vehicle_name + "/global_gps", 10);

  printf("Listening for %s\n", (vehicle_name + "/cmd_vel").c_str());
  car_ros.vel_cmd_body_frame_cb_std_sub =
      nh_private_.subscribe<geometry_msgs::Twist>(
          vehicle_name + "/cmd_vel",
          1,
          boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_std_cb,
                      this,
                      _1,
                      car_ros.vehicle_name));

  // bind to a single callback. todo optimal subs queue length
  // bind multiple topics to a single callback, but keep track of which
  // vehicle name it was by passing vehicle_name as the 2nd argument

  // TODO(Sadegh): If we need velocity cmd callbacks of these forms, add support
  //   car_ros.vel_cmd_body_frame_sub =
  //      nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(vehicle_name +
  //      "/vel_cmd_body_frame", 1,
  //       boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this, _1,
  //             car_ros.vehicle_name));
  // todo ros::TransportHints().tcpNoDelay();
  //   car_ros.vel_cmd_world_frame_sub =
  //      nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(vehicle_name +
  //     "/vel_cmd_world_frame", 1,
  //       boost::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb, this, _1,
  //       car_ros.vehicle_name));

  // TODO(Sadegh): Enable and verify the reset functionality
  //   car_ros.reset_srvr = nh_private_.advertiseService(vehicle_name +
  //     "/reset",&AirsimROSWrapper::reset_srv_cb, this);

  car_ros_vec_.push_back(car_ros);
}

void AirsimROSWrapper::add_ros_multirotor(const std::string& vehicle_name) {
  MultiRotorROS multirotor_ros;
  multirotor_ros.odom_frame_id = vehicle_name + frame_name_base_link_;
  multirotor_ros.vehicle_name = vehicle_name;
  multirotor_ros.odom_local_ned_pub =
      nh_private_.advertise<nav_msgs::Odometry>(vehicle_name + "/odom", 10);
  multirotor_ros.global_gps_pub = nh_private_.advertise<sensor_msgs::NavSatFix>(
      vehicle_name + "/global_gps", 10);

  // bind to a single callback. todo optimal subs queue length
  // bind multiple topics to a single callback, but keep track of which
  // vehicle name it was by passing vehicle_name as the 2nd argument
  multirotor_ros.vel_cmd_body_frame_sub =
      nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(
          vehicle_name + "/vel_cmd_body_frame",
          1,
          boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb,
                      this,
                      _1,
                      multirotor_ros.vehicle_name));
  // todo ros::TransportHints().tcpNoDelay();

  multirotor_ros.vel_cmd_world_frame_sub =
      nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(
          vehicle_name + "/vel_cmd_world_frame",
          1,
          boost::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb,
                      this,
                      _1,
                      multirotor_ros.vehicle_name));

  multirotor_ros.takeoff_srvr =
      nh_private_.advertiseService<airsim_ros_pkgs::Takeoff::Request,
                                   airsim_ros_pkgs::Takeoff::Response>(
          vehicle_name + "/takeoff",
          boost::bind(&AirsimROSWrapper::takeoff_srv_cb,
                      this,
                      _1,
                      _2,
                      multirotor_ros.vehicle_name));
  multirotor_ros.land_srvr =
      nh_private_.advertiseService<airsim_ros_pkgs::Land::Request,
                                   airsim_ros_pkgs::Land::Response>(
          vehicle_name + "/land",
          boost::bind(&AirsimROSWrapper::land_srv_cb,
                      this,
                      _1,
                      _2,
                      multirotor_ros.vehicle_name));

  //   multirotor_ros.reset_srvr =
  //     nh_private_.advertiseService(vehicle_name +
  //     "/reset",&AirsimROSWrapper::reset_srv_cb, this);

  multirotor_ros_vec_.push_back(multirotor_ros);
}

// XmlRpc::XmlRpcValue can't be const in this case
void AirsimROSWrapper::create_ros_pubs_from_settings_json() {
  // subscribe to control commands on global nodehandle
  if (vehicle_type_ == MULTIROTOR) {
    gimbal_angle_quat_cmd_sub_ =
        nh_private_.subscribe("gimbal_angle_quat_cmd",
                              50,
                              &AirsimROSWrapper::gimbal_angle_quat_cmd_cb,
                              this);
    gimbal_angle_euler_cmd_sub_ =
        nh_private_.subscribe("gimbal_angle_euler_cmd",
                              50,
                              &AirsimROSWrapper::gimbal_angle_euler_cmd_cb,
                              this);
  }
  origin_geo_point_pub_ =
      nh_private_.advertise<airsim_ros_pkgs::GPSYaw>("origin_geo_point", 10);

  airsim_img_request_vehicle_name_pair_vec_.clear();
  image_pub_vec_.clear();
  cam_info_pub_vec_.clear();
  camera_info_msg_vec_.clear();
  static_tf_msg_vec_.clear();
  imu_pub_vec_.clear();
  lidar_pub_vec_.clear();
  // todo should eventually support different types of vehicles in a single
  // instance
  vehicle_names_.clear();
  // vehicle_setting_vec_.clear();
  // vehicle_imu_map_;
  car_ros_vec_.clear();
  multirotor_ros_vec_.clear();
  // callback_queues_.clear();

  image_transport::ImageTransport image_transporter(nh_private_);

  // Check if simulation mode in the AirSim's settings.json
  // matches the mode AirSim's Ros Wrapper has been started
  // with
  std::string sim_mode = AirSimSettings::singleton().simmode_name;
  if (!((sim_mode == "Car" && vehicle_type_ == CAR) ||
        (sim_mode == "Multirotor" && vehicle_type_ == MULTIROTOR))) {
    LOG(FATAL) << "Sim mode that is set in the settings.json(" << sim_mode
               << ") does not match the vehicle_type set for the"
               << " ROS wrapper.";
  }

  int idx = 0;
  // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>>
  // vehicles;
  for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
    auto& vehicle_setting = curr_vehicle_elem.second;
    auto curr_vehicle_name = curr_vehicle_elem.first;
    vehicle_names_.push_back(curr_vehicle_name);
    set_nans_to_zeros_in_pose(*vehicle_setting);
    // auto vehicle_setting_local = vehicle_setting.get();

    append_static_vehicle_tf(curr_vehicle_name, *vehicle_setting);
    // allows fast lookup in command callbacks in case of a lot of drones
    vehicle_name_idx_map_[curr_vehicle_name] = idx;

    switch (vehicle_type_) {
      case CAR:
        add_ros_car(curr_vehicle_name);
        break;
      case MULTIROTOR:
        add_ros_multirotor(curr_vehicle_name);
        break;
      default:
        LOG(FATAL) << "Unknown vehicle type";
    }
    idx++;

    std::vector<ImageRequest> current_image_request_vec;
    current_image_request_vec.clear();

    // iterate over camera map std::map<std::string, CameraSetting> cameras;
    // This is an initial pass to extract all transformations between cameras
    // and detect stereo pairs
    for (auto& curr_camera_elem : vehicle_setting->cameras) {
      auto& camera_setting = curr_camera_elem.second;
      auto& curr_camera_name = curr_camera_elem.first;
      // vehicle_setting_vec_.push_back(*vehicle_setting.get());
      set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
      append_static_camera_tf(
          curr_vehicle_name, curr_camera_name, camera_setting);
    }

    // Go through the generated static tf messages and extract the
    // transormation between the stereo pair if one exists
    tf2::Transform left_cam_tf_body, right_cam_tf_body, left_cam_tf_right_cam;
    int pair_count = 0;
    for (const auto& tf_msg : static_tf_msg_vec_) {
      if (tf_msg.header.frame_id == curr_vehicle_name + frame_name_base_link_) {
        if (tf_msg.child_frame_id == "StereoLeft_body/static") {
          tf2::convert(tf_msg.transform, left_cam_tf_body);
          pair_count++;
        } else if (tf_msg.child_frame_id == "StereoRight_body/static") {
          tf2::convert(tf_msg.transform, right_cam_tf_body);
          pair_count++;
        }
      }
    }

    if (pair_count == 2) {
      left_cam_tf_right_cam = left_cam_tf_body * right_cam_tf_body.inverse();
      vehicle_name_stereo_baseline_map_[curr_vehicle_name] =
          left_cam_tf_right_cam.getOrigin().getY();
    }

    // iterate over camera map std::map<std::string, CameraSetting> cameras;
    // This is the second pass to actually generate the publishers and camera
    // info msgs
    for (auto& curr_camera_elem : vehicle_setting->cameras) {
      auto& camera_setting = curr_camera_elem.second;
      auto& curr_camera_name = curr_camera_elem.first;
      // vehicle_setting_vec_.push_back(*vehicle_setting.get());
      set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
      // camera_setting.gimbal

      // iterate over capture_setting std::map<int, CaptureSetting>
      // capture_settings
      for (const auto& curr_capture_elem : camera_setting.capture_settings) {
        auto& capture_setting = curr_capture_elem.second;

        // todo why does AirSimSettings::loadCaptureSettings calls
        // AirSimSettings::initializeCaptureSettings()
        // which initializes default capture settings for _all_ NINE
        // msr::airlib::ImageCaptureBase::ImageType
        if (!(std::isnan(capture_setting.fov_degrees))) {
          ImageType curr_image_type =
              msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
          // if scene / segmentation / surface normals / infrared, get
          // uncompressed image with pixels_as_floats = false
          if (capture_setting.image_type == 0 ||
              capture_setting.image_type == 5 ||
              capture_setting.image_type == 6 ||
              capture_setting.image_type == 7) {
            current_image_request_vec.push_back(
                ImageRequest(curr_camera_name, curr_image_type, false, false));
          }
          // if {DepthPlanner, DepthPerspective,DepthVis,
          // DisparityNormalized}, get float image
          else {
            current_image_request_vec.push_back(
                ImageRequest(curr_camera_name, curr_image_type, true));
          }

          image_pub_vec_.push_back(image_transporter.advertise(
              curr_vehicle_name + "/" + curr_camera_name + "/" +
                  image_type_int_to_string_map_.at(capture_setting.image_type),
              1));

          cam_info_pub_vec_.push_back(
              nh_private_.advertise<sensor_msgs::CameraInfo>(
                  curr_vehicle_name + "/" + curr_camera_name + "/" +
                      image_type_int_to_string_map_.at(
                          capture_setting.image_type) +
                      "/camera_info",
                  10));

          camera_info_msg_vec_.push_back(generate_cam_info(curr_vehicle_name,
                                                           curr_camera_name,
                                                           camera_setting,
                                                           capture_setting));
        }
      }
      // push back pair (vector of image captures, current vehicle name)
    }
    airsim_img_request_vehicle_name_pair_vec_.push_back(
        std::make_pair(current_image_request_vec, curr_vehicle_name));

    // iterate over sensors std::map<std::string,
    // std::unique_ptr<SensorSetting>> sensors;
    for (auto& curr_sensor_map : vehicle_setting->sensors) {
      auto& sensor_name = curr_sensor_map.first;
      auto& sensor_setting = curr_sensor_map.second;

      switch (sensor_setting->sensor_type) {
        case SensorBase::SensorType::Barometer: {
          std::cout << "Barometer" << std::endl;
          break;
        }
        case SensorBase::SensorType::Imu: {
          vehicle_imu_map_[curr_vehicle_name] = sensor_name;
          // todo this is pretty non scalable, refactor airsim and ros
          // api and maintain a vehicle <-> sensor (setting) map
          std::cout << "Imu" << std::endl;
          imu_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::Imu>(
              curr_vehicle_name + "/imu/" + sensor_name, 10));
          break;
        }
        case SensorBase::SensorType::Gps: {
          std::cout << "Gps" << std::endl;
          break;
        }
        case SensorBase::SensorType::Magnetometer: {
          std::cout << "Magnetometer" << std::endl;
          break;
        }
        case SensorBase::SensorType::Distance: {
          std::cout << "Distance" << std::endl;
          break;
        }
        case SensorBase::SensorType::Lidar: {
          std::cout << "Lidar" << std::endl;
          auto lidar_setting =
              *static_cast<LidarSetting*>(sensor_setting.get());
          // todo is there a more readable way to down-cast?
          set_nans_to_zeros_in_pose(*vehicle_setting, lidar_setting);
          append_static_lidar_tf(curr_vehicle_name, sensor_name, lidar_setting);

          // non scalable
          vehicle_lidar_map_[curr_vehicle_name] = sensor_name;
          lidar_pub_vec_.push_back(
              nh_private_.advertise<sensor_msgs::PointCloud2>(
                  curr_vehicle_name + "/lidar/" + sensor_name, 10));
          break;
        }
        default: { throw std::invalid_argument("Unexpected sensor type"); }
      }
    }
  }

  // add takeoff and land all services if more than 2 drones
  if (multirotor_ros_vec_.size() > 1) {
    takeoff_all_srvr_ = nh_private_.advertiseService(
        "all_robots/takeoff", &AirsimROSWrapper::takeoff_all_srv_cb, this);
    land_all_srvr_ = nh_private_.advertiseService(
        "all_robots/land", &AirsimROSWrapper::land_all_srv_cb, this);

    // gimbal_angle_quat_cmd_sub_ = nh_.subscribe("gimbal_angle_quat_cmd",
    // 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);

    vel_cmd_all_body_frame_sub_ =
        nh_private_.subscribe("all_robots/vel_cmd_body_frame",
                              1,
                              &AirsimROSWrapper::vel_cmd_all_body_frame_cb,
                              this);
    vel_cmd_all_world_frame_sub_ =
        nh_private_.subscribe("all_robots/vel_cmd_world_frame",
                              1,
                              &AirsimROSWrapper::vel_cmd_all_world_frame_cb,
                              this);

    vel_cmd_group_body_frame_sub_ =
        nh_private_.subscribe("group_of_robots/vel_cmd_body_frame",
                              1,
                              &AirsimROSWrapper::vel_cmd_group_body_frame_cb,
                              this);
    vel_cmd_group_world_frame_sub_ =
        nh_private_.subscribe("group_of_obots/vel_cmd_world_frame",
                              1,
                              &AirsimROSWrapper::vel_cmd_group_world_frame_cb,
                              this);

    takeoff_group_srvr_ =
        nh_private_.advertiseService("group_of_robots/takeoff",
                                     &AirsimROSWrapper::takeoff_group_srv_cb,
                                     this);
    land_group_srvr_ = nh_private_.advertiseService(
        "group_of_robots/land", &AirsimROSWrapper::land_group_srv_cb, this);
  }

  // todo add per vehicle reset in AirLib API
  reset_srvr_ = nh_private_.advertiseService(
      "reset", &AirsimROSWrapper::reset_srv_cb, this);

  reset_to_loc_srvr_ = nh_private_.advertiseService(
      "reset_to_loc", &AirsimROSWrapper::reset_to_loc_srv_cb, this);

  // todo mimic gazebo's /use_sim_time feature which publishes airsim's clock
  // time..via an rpc call?!
  // clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 10);

  // if >0 cameras, add one more thread for img_request_timer_cb
  if (airsim_img_request_vehicle_name_pair_vec_.size() > 0) {
    double update_airsim_img_response_every_n_sec;
    nh_private_.getParam("update_airsim_img_response_every_n_sec",
                         update_airsim_img_response_every_n_sec);
    bool separate_spinner = true;  // todo debugging race condition
    if (separate_spinner) {
      ros::TimerOptions timer_options(
          ros::Duration(update_airsim_img_response_every_n_sec),
          boost::bind(&AirsimROSWrapper::img_response_timer_cb, this, _1),
          &img_timer_cb_queue_);
      airsim_img_response_timer_ = nh_private_.createTimer(timer_options);
      is_used_img_timer_cb_queue_ = true;
    } else {
      airsim_img_response_timer_ = nh_private_.createTimer(
          ros::Duration(update_airsim_img_response_every_n_sec),
          &AirsimROSWrapper::img_response_timer_cb,
          this);
    }
  }

  if (lidar_pub_vec_.size() > 0) {
    double update_lidar_every_n_sec;
    nh_private_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
    // nh_private_.setCallbackQueue(&lidar_timer_cb_queue_);
    bool separate_spinner = true;  // todo debugging race condition
    if (separate_spinner) {
      ros::TimerOptions timer_options(
          ros::Duration(update_lidar_every_n_sec),
          boost::bind(&AirsimROSWrapper::lidar_timer_cb, this, _1),
          &lidar_timer_cb_queue_);
      airsim_lidar_update_timer_ = nh_private_.createTimer(timer_options);

      is_used_lidar_timer_cb_queue_ = true;
    } else {
      airsim_lidar_update_timer_ =
          nh_private_.createTimer(ros::Duration(update_lidar_every_n_sec),
                                  &AirsimROSWrapper::lidar_timer_cb,
                                  this);
    }
  }

  initialize_airsim();
}

ros::Time AirsimROSWrapper::make_ts(uint64_t unreal_ts) {
  if (first_imu_unreal_ts < 0) {
    first_imu_unreal_ts = unreal_ts;
    first_imu_ros_ts = ros::Time::now();
  }
  return first_imu_ros_ts +
         ros::Duration((unreal_ts - first_imu_unreal_ts) / 1e9);
}

// todo: error check. if state is not landed, return error.
bool AirsimROSWrapper::takeoff_srv_cb(
    airsim_ros_pkgs::Takeoff::Request& request,
    airsim_ros_pkgs::Takeoff::Response& response,
    const std::string& vehicle_name) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    // todo value for timeout_sec?
    airsim_client_.takeoffAsync(20, vehicle_name)->waitOnLastTask();

  // response.success =
  else
    airsim_client_.takeoffAsync(20, vehicle_name);
  // response.success =
  return true;
}

bool AirsimROSWrapper::takeoff_group_srv_cb(
    airsim_ros_pkgs::TakeoffGroup::Request& request,
    airsim_ros_pkgs::TakeoffGroup::Response& response) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    for (const auto& vehicle_name : request.vehicle_names)
      // todo value for timeout_sec?
      airsim_client_.takeoffAsync(20, vehicle_name)->waitOnLastTask();
  // response.success =
  else
    for (const auto& vehicle_name : request.vehicle_names)
      airsim_client_.takeoffAsync(20, vehicle_name);
  // response.success =
  return true;
}

bool AirsimROSWrapper::takeoff_all_srv_cb(
    airsim_ros_pkgs::Takeoff::Request& request,
    airsim_ros_pkgs::Takeoff::Response& response) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    for (const auto& vehicle_name : vehicle_names_)
      // todo value for timeout_sec?
      airsim_client_.takeoffAsync(20, vehicle_name)->waitOnLastTask();
  // response.success =
  else
    for (const auto& vehicle_name : vehicle_names_)
      airsim_client_.takeoffAsync(20, vehicle_name);
  // response.success =
  return true;
}

bool AirsimROSWrapper::land_srv_cb(airsim_ros_pkgs::Land::Request& request,
                                   airsim_ros_pkgs::Land::Response& response,
                                   const std::string& vehicle_name) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    airsim_client_.landAsync(60, vehicle_name)->waitOnLastTask();
  else
    airsim_client_.landAsync(60, vehicle_name);
  return true;  // todo
}

bool AirsimROSWrapper::land_group_srv_cb(
    airsim_ros_pkgs::LandGroup::Request& request,
    airsim_ros_pkgs::LandGroup::Response& response) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    for (const auto& vehicle_name : request.vehicle_names)
      airsim_client_.landAsync(60, vehicle_name)->waitOnLastTask();
  else
    for (const auto& vehicle_name : request.vehicle_names)
      airsim_client_.landAsync(60, vehicle_name);
  return true;  // todo
}

bool AirsimROSWrapper::land_all_srv_cb(
    airsim_ros_pkgs::Land::Request& request,
    airsim_ros_pkgs::Land::Response& response) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (request.waitOnLastTask)
    for (const auto& vehicle_name : vehicle_names_)
      airsim_client_.landAsync(60, vehicle_name)->waitOnLastTask();
  else
    for (const auto& vehicle_name : vehicle_names_)
      airsim_client_.landAsync(60, vehicle_name);
  return true;  // todo
}

// todo add reset by vehicle_name API to airlib
// todo not async remove waitonlasttask
bool AirsimROSWrapper::reset_srv_cb(
    airsim_ros_pkgs::Reset::Request& request,
    airsim_ros_pkgs::Reset::Response& response) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);
  switch (vehicle_type_) {
    case CAR:
      airsim_car_client_.reset();
      break;
    case MULTIROTOR:
      airsim_client_.reset();
      break;
    default:
      LOG(FATAL) << "Unknown vehicle type";
  }

  return true;  // todo
}

bool AirsimROSWrapper::reset_to_loc_srv_cb(
    airsim_ros_pkgs::ResetToLocation::Request& request,
    airsim_ros_pkgs::ResetToLocation::Response& response) {
  // TODO: Make sure all attributes of car_ros are reset (e.g. collision)
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  if (vehicle_name_idx_map_.find(request.vehicle_name) ==
      vehicle_name_idx_map_.end()) {
    LOG(WARNING) << "Vehicle " << request.vehicle_name << " was not found!";
    response.success = false;
    return false;
  }

  if (vehicle_type_ != CAR) {
    LOG(WARNING) << "Resetting to location service is currently only supported"
                 << " for cars.";
    response.success = false;
    return false;
  }

  // If the NWU coordinate frame is being used (the ROS standard), it is
  // assumed that the requested pose is also in NWU. It should be converted
  // to NED before being passed to the AirSim API.
  geometry_msgs::Pose target_pose;
  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::Transform tf_OdomNED_BaseNED, tf_OdomNWU_BaseNWU;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);
    tf2::convert(request.pose, tf_OdomNWU_BaseNWU);

    tf_OdomNED_BaseNED = tf_ned_nwu * tf_OdomNWU_BaseNWU * tf_nwu_ned;

    target_pose.position.x = tf_OdomNED_BaseNED.getOrigin().getX();
    target_pose.position.y = tf_OdomNED_BaseNED.getOrigin().getY();
    target_pose.position.z = tf_OdomNED_BaseNED.getOrigin().getZ();
    target_pose.orientation = tf2::toMsg(tf_OdomNED_BaseNED.getRotation());
  } else {
    target_pose = request.pose;
  }

  Eigen::Vector3f position(
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Quaternionf orientation(target_pose.orientation.w,
                                 target_pose.orientation.x,
                                 target_pose.orientation.y,
                                 target_pose.orientation.z);
  Pose pose(position, orientation);
  airsim_car_client_.simSetVehiclePose(pose, true, request.vehicle_name);

  response.success = true;
  return true;
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(
    const msr::airlib::Quaternionr& airlib_quat) const {
  return tf2::Quaternion(
      airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(
    const geometry_msgs::Quaternion& geometry_msgs_quat) const {
  return msr::airlib::Quaternionr(geometry_msgs_quat.w,
                                  geometry_msgs_quat.x,
                                  geometry_msgs_quat.y,
                                  geometry_msgs_quat.z);
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(
    const tf2::Quaternion& tf2_quat) const {
  return msr::airlib::Quaternionr(
      tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}

void AirsimROSWrapper::vel_cmd_body_frame_std_cb(
    const geometry_msgs::Twist::ConstPtr& msg,
    const std::string& vehicle_name) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  int vehicle_idx = vehicle_name_idx_map_[vehicle_name];
  car_ros_vec_[vehicle_idx].vel_cmd.t = ros::Time::now();
  car_ros_vec_[vehicle_idx].vel_cmd.x = msg->linear.x;
  car_ros_vec_[vehicle_idx].vel_cmd.y = msg->linear.y;
  car_ros_vec_[vehicle_idx].vel_cmd.z = msg->linear.z;
  car_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
      msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  car_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
  car_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate = msg->angular.z;
  car_ros_vec_[vehicle_idx].velocity_controller->set_target(
      car_ros_vec_[vehicle_idx].vel_cmd);
  car_ros_vec_[vehicle_idx].has_vel_cmd = true;
}

// TODO(Sadegh): Update this to support cars
// void AirsimROSWrapper::vel_cmd_body_frame_cb(const
// airsim_ros_pkgs::VelCmd& msg, const std::string&
// vehicle_name)
void AirsimROSWrapper::vel_cmd_body_frame_cb(
    const airsim_ros_pkgs::VelCmd::ConstPtr& msg,
    const std::string& vehicle_name) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

  double roll, pitch, yaw;
  // ros uses xyzw
  tf2::Matrix3x3(
      get_tf2_quat(multirotor_ros_vec_[vehicle_idx]
                       .curr_drone_state.kinematics_estimated.pose.orientation))
      .getRPY(roll, pitch, yaw);

  // todo do actual body frame?
  // body frame assuming zero pitch roll
  multirotor_ros_vec_[vehicle_idx].vel_cmd.x =
      (msg->twist.linear.x * cos(yaw)) - (msg->twist.linear.y * sin(yaw));
  multirotor_ros_vec_[vehicle_idx].vel_cmd.y =
      (msg->twist.linear.x * sin(yaw)) +
      (msg->twist.linear.y * cos(yaw));  // body frame
  multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg->twist.linear.z;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
      msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
  // airsim uses degrees
  multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
      math_common::rad2deg(msg->twist.angular.z);
  multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
}

// TODO(Sadegh): Update this to support cars
void AirsimROSWrapper::vel_cmd_group_body_frame_cb(
    const airsim_ros_pkgs::VelCmdGroup& msg) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  for (const auto& vehicle_name : msg.vehicle_names) {
    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];
    double roll, pitch, yaw;
    // ros uses xyzw
    tf2::Matrix3x3(
        get_tf2_quat(
            multirotor_ros_vec_[vehicle_idx]
                .curr_drone_state.kinematics_estimated.pose.orientation))
        .getRPY(roll, pitch, yaw);

    // todo do actual body frame?
    // body frame assuming zero pitch roll
    multirotor_ros_vec_[vehicle_idx].vel_cmd.x =
        (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw));
    multirotor_ros_vec_[vehicle_idx].vel_cmd.y =
        (msg.twist.linear.x * sin(yaw)) +
        (msg.twist.linear.y * cos(yaw));  // body frame
    multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg.twist.linear.z;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
    // airsim uses degrees
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
        math_common::rad2deg(msg.twist.angular.z);
    multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
  }
}

// TODO(Sadegh): Update this to support cars
// void AirsimROSWrapper::vel_cmd_all_body_frame_cb(const
// airsim_ros_pkgs::VelCmd::ConstPtr& msg)
void AirsimROSWrapper::vel_cmd_all_body_frame_cb(
    const airsim_ros_pkgs::VelCmd& msg) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  // todo expose waitOnLastTask or nah?
  for (const auto& vehicle_name : vehicle_names_) {
    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];
    double roll, pitch, yaw;
    // ros uses xyzw
    tf2::Matrix3x3(
        get_tf2_quat(
            multirotor_ros_vec_[vehicle_idx]
                .curr_drone_state.kinematics_estimated.pose.orientation))
        .getRPY(roll, pitch, yaw);

    // todo do actual body frame?
    // body frame assuming zero pitch roll
    multirotor_ros_vec_[vehicle_idx].vel_cmd.x =
        (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw));
    multirotor_ros_vec_[vehicle_idx].vel_cmd.y =
        (msg.twist.linear.x * sin(yaw)) +
        (msg.twist.linear.y * cos(yaw));  // body frame
    multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg.twist.linear.z;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
    // airsim uses degrees
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
        math_common::rad2deg(msg.twist.angular.z);
    multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
  }
}

// TODO(Sadegh): Update this to support cars
void AirsimROSWrapper::vel_cmd_world_frame_cb(
    const airsim_ros_pkgs::VelCmd::ConstPtr& msg,
    const std::string& vehicle_name) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

  multirotor_ros_vec_[vehicle_idx].vel_cmd.x = msg->twist.linear.x;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.y = msg->twist.linear.y;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg->twist.linear.z;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
      msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
  multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
      math_common::rad2deg(msg->twist.angular.z);
  multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
}

// TODO(Sadegh): Update this to support cars
// this is kinda unnecessary but maybe it makes life easier for
// the end user.
void AirsimROSWrapper::vel_cmd_group_world_frame_cb(
    const airsim_ros_pkgs::VelCmdGroup& msg) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  for (const auto& vehicle_name : msg.vehicle_names) {
    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

    multirotor_ros_vec_[vehicle_idx].vel_cmd.x = msg.twist.linear.x;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.y = msg.twist.linear.y;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg.twist.linear.z;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
    multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
        math_common::rad2deg(msg.twist.angular.z);
    multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
  }
}

void AirsimROSWrapper::vel_cmd_all_world_frame_cb(
    const airsim_ros_pkgs::VelCmd& msg) {
  std::lock_guard<std::mutex> guard(drone_control_mutex_);

  switch (vehicle_type_) {
    case CAR:
      // todo expose waitOnLastTask or nah?
      for (const auto& vehicle_name : vehicle_names_) {
        int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

        car_ros_vec_[vehicle_idx].vel_cmd.x = msg.twist.linear.x;
        car_ros_vec_[vehicle_idx].vel_cmd.y = msg.twist.linear.y;
        car_ros_vec_[vehicle_idx].vel_cmd.z = msg.twist.linear.z;
        car_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        car_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
        car_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
            math_common::rad2deg(msg.twist.angular.z);
        car_ros_vec_[vehicle_idx].has_vel_cmd = true;
        //   printf("Recieved vel cmd\n");
      }
      break;
    case MULTIROTOR:
      // todo expose waitOnLastTask or nah?
      for (const auto& vehicle_name : vehicle_names_) {
        int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

        multirotor_ros_vec_[vehicle_idx].vel_cmd.x = msg.twist.linear.x;
        multirotor_ros_vec_[vehicle_idx].vel_cmd.y = msg.twist.linear.y;
        multirotor_ros_vec_[vehicle_idx].vel_cmd.z = msg.twist.linear.z;
        multirotor_ros_vec_[vehicle_idx].vel_cmd.drivetrain =
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.is_rate = true;
        multirotor_ros_vec_[vehicle_idx].vel_cmd.yaw_mode.yaw_or_rate =
            math_common::rad2deg(msg.twist.angular.z);
        multirotor_ros_vec_[vehicle_idx].has_vel_cmd = true;
      }
      break;
    default:
      LOG(FATAL) << "Unknown vehicle type";
  }
}

// TODO(Sadegh): Add a ROS subscriber for standard velocity
// commands

// todo support multiple gimbal commands
void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(
    const airsim_ros_pkgs::GimbalAngleQuatCmd& gimbal_angle_quat_cmd_msg) {
  tf2::Quaternion quat_control_cmd;
  try {
    tf2::convert(gimbal_angle_quat_cmd_msg.orientation, quat_control_cmd);
    quat_control_cmd.normalize();
    // airsim uses wxyz
    gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
    gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg.camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg.vehicle_name;
    has_gimbal_cmd_ = true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
}

// todo support multiple gimbal commands
// 1. find quaternion of default gimbal pose
// 2. forward multiply with quaternion equivalent to desired
// euler commands (in degrees)
// 3. call airsim client's setcameraorientation which sets
// camera orientation wrt world (or takeoff?) ned frame. todo
void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(
    const airsim_ros_pkgs::GimbalAngleEulerCmd& gimbal_angle_euler_cmd_msg) {
  try {
    tf2::Quaternion quat_control_cmd;

    quat_control_cmd.setRPY(
        math_common::deg2rad(gimbal_angle_euler_cmd_msg.roll),
        math_common::deg2rad(gimbal_angle_euler_cmd_msg.pitch),
        math_common::deg2rad(gimbal_angle_euler_cmd_msg.yaw));
    quat_control_cmd.normalize();
    gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
    gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg.camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg.vehicle_name;
    has_gimbal_cmd_ = true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_airsim_state(
    const msr::airlib::MultirotorState& drone_state) const {
  nav_msgs::Odometry odom_msg;
  // odom_msg.header.frame_id = world_frame_id_;
  // odom_msg.child_frame_id = "/airsim/odom_local_ned"; // todo
  // make param

  odom_msg.pose.pose.position.x = drone_state.getPosition().x();
  odom_msg.pose.pose.position.y = drone_state.getPosition().y();
  odom_msg.pose.pose.position.z = drone_state.getPosition().z();
  odom_msg.pose.pose.orientation.x = drone_state.getOrientation().x();
  odom_msg.pose.pose.orientation.y = drone_state.getOrientation().y();
  odom_msg.pose.pose.orientation.z = drone_state.getOrientation().z();
  odom_msg.pose.pose.orientation.w = drone_state.getOrientation().w();

  odom_msg.twist.twist.linear.x =
      drone_state.kinematics_estimated.twist.linear.x();
  odom_msg.twist.twist.linear.y =
      drone_state.kinematics_estimated.twist.linear.y();
  odom_msg.twist.twist.linear.z =
      drone_state.kinematics_estimated.twist.linear.z();
  odom_msg.twist.twist.angular.x =
      drone_state.kinematics_estimated.twist.angular.x();
  odom_msg.twist.twist.angular.y =
      drone_state.kinematics_estimated.twist.angular.y();
  odom_msg.twist.twist.angular.z =
      drone_state.kinematics_estimated.twist.angular.z();

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::Transform tf_OdomNED_BaseNED, tf_OdomNWU_BaseNWU;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);
    tf2::convert(odom_msg.pose.pose, tf_OdomNED_BaseNED);

    tf_OdomNWU_BaseNWU = tf_nwu_ned * tf_OdomNED_BaseNED * tf_ned_nwu;

    odom_msg.pose.pose.position.x = tf_OdomNWU_BaseNWU.getOrigin().getX();
    odom_msg.pose.pose.position.y = tf_OdomNWU_BaseNWU.getOrigin().getY();
    odom_msg.pose.pose.position.z = tf_OdomNWU_BaseNWU.getOrigin().getZ();
    odom_msg.pose.pose.orientation =
        tf2::toMsg(tf_OdomNWU_BaseNWU.getRotation());

    // Convert the twist messages. The twist messages first need
    // to be converted from NED to NWU. Then they are
    // transformed from the odom frame to the base_link frame
    // (child_frame of the odometry msg) as this is the standard
    // given ROS documentation for the odometry message.
    geometry_msgs::TransformStamped odom_inv_tf_msg;
    tf2::convert(tf_OdomNWU_BaseNWU.inverse(), odom_inv_tf_msg.transform);

    tf2::doTransform(odom_msg.twist.twist.linear,
                     odom_msg.twist.twist.linear,
                     trans_nwu_ned_);
    tf2::doTransform(odom_msg.twist.twist.linear,
                     odom_msg.twist.twist.linear,
                     odom_inv_tf_msg);

    tf2::doTransform(odom_msg.twist.twist.angular,
                     odom_msg.twist.twist.angular,
                     trans_nwu_ned_);
    tf2::doTransform(odom_msg.twist.twist.angular,
                     odom_msg.twist.twist.angular,
                     odom_inv_tf_msg);
  }

  return odom_msg;
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_airsim_state(
    const msr::airlib::CarApiBase::CarState& car_state) const {
  nav_msgs::Odometry odom_msg;
  // odom_msg.header.frame_id = world_frame_id_;
  // todo make param
  // odom_msg.child_frame_id = "/airsim/odom_local_ned";
  const msr::airlib_rpclib::RpcLibAdapatorsBase::KinematicsState& kinematics =
      car_state.kinematics_estimated;

  odom_msg.pose.pose.position.x = kinematics.position.x_val;
  odom_msg.pose.pose.position.y = kinematics.position.y_val;
  odom_msg.pose.pose.position.z = kinematics.position.z_val;
  odom_msg.pose.pose.orientation.x = kinematics.orientation.x_val;
  odom_msg.pose.pose.orientation.y = kinematics.orientation.y_val;
  odom_msg.pose.pose.orientation.z = kinematics.orientation.z_val;
  odom_msg.pose.pose.orientation.w = kinematics.orientation.w_val;

  odom_msg.twist.twist.linear.x = kinematics.linear_velocity.x_val;
  odom_msg.twist.twist.linear.y = kinematics.linear_velocity.y_val;
  odom_msg.twist.twist.linear.z = kinematics.linear_velocity.z_val;
  odom_msg.twist.twist.angular.x = kinematics.angular_velocity.x_val;
  odom_msg.twist.twist.angular.y = kinematics.angular_velocity.y_val;
  odom_msg.twist.twist.angular.z = kinematics.angular_velocity.z_val;

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::Transform tf_OdomNED_BaseNED, tf_OdomNWU_BaseNWU;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);
    tf2::convert(odom_msg.pose.pose, tf_OdomNED_BaseNED);

    tf_OdomNWU_BaseNWU = tf_nwu_ned * tf_OdomNED_BaseNED * tf_ned_nwu;

    odom_msg.pose.pose.position.x = tf_OdomNWU_BaseNWU.getOrigin().getX();
    odom_msg.pose.pose.position.y = tf_OdomNWU_BaseNWU.getOrigin().getY();
    odom_msg.pose.pose.position.z = tf_OdomNWU_BaseNWU.getOrigin().getZ();
    odom_msg.pose.pose.orientation =
        tf2::toMsg(tf_OdomNWU_BaseNWU.getRotation());

    // Convert the twist messages. The twist messages first need
    // to be converted from NED to NWU. Then they are
    // transformed from the odom frame to the base_link frame
    // (child_frame of the odometry msg) as this is the standard
    // given ROS documentation for the odometry message.
    geometry_msgs::TransformStamped odom_inv_tf_msg;
    tf2::convert(tf_OdomNWU_BaseNWU.inverse(), odom_inv_tf_msg.transform);

    tf2::doTransform(odom_msg.twist.twist.linear,
                     odom_msg.twist.twist.linear,
                     trans_nwu_ned_);
    tf2::doTransform(odom_msg.twist.twist.linear,
                     odom_msg.twist.twist.linear,
                     odom_inv_tf_msg);

    tf2::doTransform(odom_msg.twist.twist.angular,
                     odom_msg.twist.twist.angular,
                     trans_nwu_ned_);
    tf2::doTransform(odom_msg.twist.twist.angular,
                     odom_msg.twist.twist.angular,
                     odom_inv_tf_msg);
  }

  return odom_msg;
}

airsim_ros_pkgs::CollisionInfo
AirsimROSWrapper::get_collision_msg_from_airsim_info(
    const msr::airlib::CollisionInfo& collision_info,
    const std::string& frame_id) const {
  airsim_ros_pkgs::CollisionInfo collision_msg;
  collision_msg.header.stamp =
      airsim_timestamp_to_ros(collision_info.time_stamp);
  collision_msg.header.frame_id = frame_id;
  collision_msg.has_collided = collision_info.has_collided;
  collision_msg.object_name = collision_info.object_name;

  return collision_msg;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/
// point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp
// UnrealLidarSensor::getPointCloud() for math read this
// carefully
// https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 AirsimROSWrapper::get_lidar_msg_from_airsim(
    const msr::airlib::LidarData& lidar_data) {
  sensor_msgs::PointCloud2 lidar_msg;
  lidar_msg.header.frame_id = world_frame_id_;  // todo
  lidar_msg.header.stamp = airsim_timestamp_to_ros(lidar_data.time_stamp);

  if (lidar_data.point_cloud.size() > 3) {
    lidar_msg.height = 1;
    lidar_msg.width = lidar_data.point_cloud.size() / 3;

    lidar_msg.fields.resize(3);
    lidar_msg.fields[0].name = "x";
    lidar_msg.fields[1].name = "y";
    lidar_msg.fields[2].name = "z";
    int offset = 0;

    for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
      lidar_msg.fields[d].offset = offset;
      lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      lidar_msg.fields[d].count = 1;
    }

    lidar_msg.is_bigendian = false;
    lidar_msg.point_step = offset;  // 4 * num fields
    lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

    lidar_msg.is_dense = true;  // todo
    std::vector<float> data_std = lidar_data.point_cloud;

    const unsigned char* bytes =
        reinterpret_cast<const unsigned char*>(&data_std[0]);
    vector<unsigned char> lidar_msg_data(
        bytes, bytes + sizeof(float) * data_std.size());
    lidar_msg.data = std::move(lidar_msg_data);
  } else {
    // msg = []
  }
  return lidar_msg;
}

// todo covariances
sensor_msgs::Imu AirsimROSWrapper::get_imu_msg_from_airsim(
    const msr::airlib::ImuBase::Output& imu_data) {
  sensor_msgs::Imu imu_msg;
  // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo
  // multiple drones
  imu_msg.orientation.x = imu_data.orientation.x();
  imu_msg.orientation.y = imu_data.orientation.y();
  imu_msg.orientation.z = imu_data.orientation.z();
  imu_msg.orientation.w = imu_data.orientation.w();

  // todo radians per second
  imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
  imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
  imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

  // meters/s2^m
  imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
  imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
  imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

  imu_msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp);
  // imu_msg.orientation_covariance = ;
  // imu_msg.angular_velocity_covariance = ;
  // imu_msg.linear_acceleration_covariance = ;

  return imu_msg;
}

void AirsimROSWrapper::publish_odom_tf(const nav_msgs::Odometry& odom_msg) {
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header = odom_msg.header;
  odom_tf.child_frame_id = odom_msg.child_frame_id;
  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;

  tf_broadcaster_.sendTransform(odom_tf);
}

airsim_ros_pkgs::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(
    const msr::airlib::GeoPoint& geo_point) const {
  airsim_ros_pkgs::GPSYaw gps_msg;
  gps_msg.latitude = geo_point.latitude;
  gps_msg.longitude = geo_point.longitude;
  gps_msg.altitude = geo_point.altitude;
  return gps_msg;
}

sensor_msgs::NavSatFix
AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(
    const msr::airlib::GeoPoint& geo_point) const {
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.latitude = geo_point.latitude;
  gps_msg.longitude = geo_point.longitude;
  gps_msg.altitude = geo_point.altitude;
  return gps_msg;
}

// todo unused
// void AirsimROSWrapper::set_zero_vel_cmd()
// {
//     vel_cmd_.x = 0.0;
//     vel_cmd_.y = 0.0;
//     vel_cmd_.z = 0.0;

//     vel_cmd_.drivetrain =
//     msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
//     vel_cmd_.yaw_mode.is_rate = false;

//     // todo make class member or a fucntion
//     double roll, pitch, yaw;
//
//     tf2::Matrix3x3(get_tf2_quat(curr_drone_state_.kinematics_estimated.pose.
//      orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw
//     vel_cmd_.yaw_mode.yaw_or_rate = yaw;
// }

ros::Time AirsimROSWrapper::chrono_timestamp_to_ros(
    const std::chrono::system_clock::time_point& stamp) const {
  auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
  ros::Time cur_time;
  cur_time.fromSec(dur.count());
  return cur_time;
}

ros::Time AirsimROSWrapper::airsim_timestamp_to_ros(
    const msr::airlib::TTimePoint& stamp) const {
  // airsim appears to use chrono::system_clock with nanosecond
  // precision
  std::chrono::nanoseconds dur(stamp);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  ros::Time cur_time = chrono_timestamp_to_ros(tp);
  return cur_time;
}

void AirsimROSWrapper::drone_state_timer_cb(const ros::TimerEvent& event) {
  try {
    // todo this is global origin
    origin_geo_point_pub_.publish(origin_geo_point_msg_);
    // iterate over drones
    for (auto& multirotor_ros : multirotor_ros_vec_) {
      // get drone state from airsim
      multirotor_ros.curr_drone_state =
          airsim_client_.getMultirotorState(multirotor_ros.vehicle_name);
      ros::Time curr_ros_time = ros::Time::now();

      // convert airsim drone state to ROS msgs
      multirotor_ros.curr_odom =
          get_odom_msg_from_airsim_state(multirotor_ros.curr_drone_state);
      multirotor_ros.curr_odom.header.frame_id = multirotor_ros.vehicle_name;
      multirotor_ros.curr_odom.child_frame_id = multirotor_ros.odom_frame_id;
      multirotor_ros.curr_odom.header.stamp = curr_ros_time;

      multirotor_ros.gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(
          multirotor_ros.curr_drone_state.gps_location);
      multirotor_ros.gps_sensor_msg.header.stamp = curr_ros_time;

      // publish to ROS!

      multirotor_ros.odom_local_ned_pub.publish(multirotor_ros.curr_odom);
      publish_odom_tf(multirotor_ros.curr_odom);

      multirotor_ros.global_gps_pub.publish(multirotor_ros.gps_sensor_msg);

      // send control commands from the last callback to airsim
      if (multirotor_ros.has_vel_cmd && use_api_control_) {
        std::lock_guard<std::mutex> guard(drone_control_mutex_);
        airsim_client_.moveByVelocityAsync(
            multirotor_ros.vel_cmd.x,
            multirotor_ros.vel_cmd.y,
            multirotor_ros.vel_cmd.z,
            vel_cmd_duration_,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            multirotor_ros.vel_cmd.yaw_mode,
            multirotor_ros.vehicle_name);
      }

      // "clear" control cmds
      multirotor_ros.has_vel_cmd = false;
    }

    // IMUS
    if (imu_pub_vec_.size() > 0) {
      int ctr = 0;
      for (const auto& vehicle_imu_pair : vehicle_imu_map_) {
        auto imu_data = airsim_client_.getImuData(vehicle_imu_pair.second,
                                                  vehicle_imu_pair.first);
        sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
        imu_msg.header.frame_id = vehicle_imu_pair.first;
        // imu_msg.header.stamp = ros::Time::now();
        imu_pub_vec_[ctr].publish(imu_msg);
        ctr++;
      }
    }

    if (static_tf_msg_vec_.size() > 0) {
      for (auto& static_tf_msg : static_tf_msg_vec_) {
        static_tf_msg.header.stamp = ros::Time::now();
        static_tf_pub_.sendTransform(static_tf_msg);
      }

      // we've sent these static transforms, so no need to keep
      // sending them
      static_tf_msg_vec_.clear();
    }

    // todo add and expose a gimbal angular velocity to airlib
    if (has_gimbal_cmd_) {
      std::lock_guard<std::mutex> guard(drone_control_mutex_);
      airsim_client_.simSetCameraOrientation(gimbal_cmd_.camera_name,
                                             gimbal_cmd_.target_quat,
                                             gimbal_cmd_.vehicle_name);
    }

    has_gimbal_cmd_ = false;
  }

  catch (rpc::rpc_error& e) {
    std::cout << "error" << std::endl;
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API:" << std::endl
              << msg << std::endl;
  }
}

const double VEL_CMD_DURATION = 15.0;

void AirsimROSWrapper::car_state_timer_cb(const ros::TimerEvent& event) {
  try {
    // todo this is global origin
    origin_geo_point_pub_.publish(origin_geo_point_msg_);
    // iterate over drones
    for (auto& car_ros : car_ros_vec_) {
      // get drone state from airsim
      car_ros.curr_car_state =
          airsim_car_client_.getCarState(car_ros.vehicle_name);
      msr::airlib::CollisionInfo collision_info =
          airsim_car_client_.simGetCollisionInfo(car_ros.vehicle_name);
      ros::Time curr_ros_time = ros::Time::now();

      if (apply_smoothing_to_odom_) {
        car_ros.ang_vel_z_smooth.AddMeasurement(
            car_ros.curr_car_state.kinematics_estimated.twist.angular.z());
        car_ros.curr_car_state.kinematics_estimated.twist.angular.z() =
            car_ros.ang_vel_z_smooth.GetEstimate();
      }

      // convert airsim drone state to ROS msgs
      car_ros.curr_odom =
          get_odom_msg_from_airsim_state(car_ros.curr_car_state);
      car_ros.curr_odom.header.frame_id = car_ros.vehicle_name;
      car_ros.curr_odom.child_frame_id = car_ros.odom_frame_id;
      car_ros.curr_odom.header.stamp = curr_ros_time;

      // TODO(Sadegh): Get GPS readings for the car. There is no
      // GPS data in the car state
      //         car_ros.gps_sensor_msg =
      //             get_gps_sensor_msg_from_airsim_geo_point(
      //                                     airsim_car_client_.getGpsData());
      //         car_ros.gps_sensor_msg.header.stamp =
      //         curr_ros_time;

      // publish to ROS!

      car_ros.odom_local_ned_pub.publish(car_ros.curr_odom);
      publish_odom_tf(car_ros.curr_odom);

      // convert collision info to ROS msgs
      if (collision_info.has_collided &&
          (car_ros.curr_collision_msg.header.stamp <
               airsim_timestamp_to_ros(collision_info.time_stamp) ||
           !car_ros.curr_collision_msg.has_collided)) {
        car_ros.curr_collision_msg = get_collision_msg_from_airsim_info(
            collision_info, car_ros.odom_frame_id);
        car_ros.collision_info_pub.publish(car_ros.curr_collision_msg);
      }

      //         car_ros.global_gps_pub.publish(car_ros.gps_sensor_msg);

      if (use_api_control_) {
        if (car_ros.has_vel_cmd &&
            (ros::Time::now() - car_ros.vel_cmd.t).toSec() > VEL_CMD_DURATION) {
          car_ros.has_vel_cmd = false;
          car_ros.velocity_controller->set_zero_target();
          car_ros.velocity_controller->set_zero_target();
          car_ros.velocity_controller->set_zero_target();
        }

        // send control commands from the last callback to
        // airsim
        CarApiBase::CarControls controls =
            car_ros.velocity_controller->get_next(
                car_ros.curr_car_state.kinematics_estimated.twist,
                car_ros.curr_car_state.speed,
                ros::Time::now());
        std::lock_guard<std::mutex> guard(drone_control_mutex_);
        airsim_car_client_.setCarControls(controls);
        airsim_car_client_.setCarControls(controls);
        airsim_car_client_.setCarControls(controls);
      }
    }

    // IMUS
    if (imu_pub_vec_.size() > 0) {
      int ctr = 0;
      for (const auto& vehicle_imu_pair : vehicle_imu_map_) {
        auto imu_data = airsim_car_client_.getImuData(vehicle_imu_pair.second,
                                                      vehicle_imu_pair.first);
        sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
        imu_msg.header.frame_id = vehicle_imu_pair.first;
        // imu_msg.header.stamp = ros::Time::now();
        imu_pub_vec_[ctr].publish(imu_msg);
        ctr++;
      }
    }

    if (static_tf_msg_vec_.size() > 0) {
      for (auto& static_tf_msg : static_tf_msg_vec_) {
        static_tf_msg.header.stamp = ros::Time::now();
        static_tf_pub_.sendTransform(static_tf_msg);
      }

      // we've sent these static transforms, so no need to keep
      // sending them
      static_tf_msg_vec_.clear();
    }
  }

  catch (rpc::rpc_error& e) {
    std::cout << "error" << std::endl;
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API:" << std::endl
              << msg << std::endl;
  }
}

// airsim uses nans for zeros in settings.json. we set them to
// zeros here for handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(
    VehicleSetting& vehicle_setting) const {
  if (std::isnan(vehicle_setting.position.x()))
    vehicle_setting.position.x() = 0.0;

  if (std::isnan(vehicle_setting.position.y()))
    vehicle_setting.position.y() = 0.0;

  if (std::isnan(vehicle_setting.position.z()))
    vehicle_setting.position.z() = 0.0;

  if (std::isnan(vehicle_setting.rotation.yaw))
    vehicle_setting.rotation.yaw = 0.0;

  if (std::isnan(vehicle_setting.rotation.pitch))
    vehicle_setting.rotation.pitch = 0.0;

  if (std::isnan(vehicle_setting.rotation.roll))
    vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose
// (which has already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(
    const VehicleSetting& vehicle_setting,
    CameraSetting& camera_setting) const {
  if (std::isnan(camera_setting.position.x()))
    camera_setting.position.x() = vehicle_setting.position.x();

  if (std::isnan(camera_setting.position.y()))
    camera_setting.position.y() = vehicle_setting.position.y();

  if (std::isnan(camera_setting.position.z()))
    camera_setting.position.z() = vehicle_setting.position.z();

  if (std::isnan(camera_setting.rotation.yaw))
    camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

  if (std::isnan(camera_setting.rotation.pitch))
    camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

  if (std::isnan(camera_setting.rotation.roll))
    camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::set_nans_to_zeros_in_pose(
    const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const {
  if (std::isnan(lidar_setting.position.x()))
    lidar_setting.position.x() = vehicle_setting.position.x();

  if (std::isnan(lidar_setting.position.y()))
    lidar_setting.position.y() = vehicle_setting.position.y();

  if (std::isnan(lidar_setting.position.z()))
    lidar_setting.position.z() = vehicle_setting.position.z();

  if (std::isnan(lidar_setting.rotation.yaw))
    lidar_setting.rotation.yaw = vehicle_setting.rotation.yaw;

  if (std::isnan(lidar_setting.rotation.pitch))
    lidar_setting.rotation.pitch = vehicle_setting.rotation.pitch;

  if (std::isnan(lidar_setting.rotation.roll))
    lidar_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::append_static_vehicle_tf(
    const std::string& vehicle_name, const VehicleSetting& vehicle_setting) {
  geometry_msgs::TransformStamped vehicle_tf_msg;
  vehicle_tf_msg.header.frame_id = world_frame_id_;
  vehicle_tf_msg.header.stamp = ros::Time::now();
  vehicle_tf_msg.child_frame_id = vehicle_name;
  vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
  vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
  vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
  tf2::Quaternion quat;
  quat.setRPY(vehicle_setting.rotation.roll,
              vehicle_setting.rotation.pitch,
              vehicle_setting.rotation.yaw);
  vehicle_tf_msg.transform.rotation.x = quat.x();
  vehicle_tf_msg.transform.rotation.y = quat.y();
  vehicle_tf_msg.transform.rotation.z = quat.z();
  vehicle_tf_msg.transform.rotation.w = quat.w();

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::Transform tf_WorldNED_BaseNED, tf_WorldNWU_BaseNWU;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);
    tf2::convert(vehicle_tf_msg.transform, tf_WorldNED_BaseNED);

    tf_WorldNWU_BaseNWU = tf_nwu_ned * tf_WorldNED_BaseNED * tf_ned_nwu;
    tf2::convert(tf_WorldNWU_BaseNWU, vehicle_tf_msg.transform);
  }

  static_tf_msg_vec_.push_back(vehicle_tf_msg);
}

void AirsimROSWrapper::append_static_lidar_tf(
    const std::string& vehicle_name,
    const std::string& lidar_name,
    const LidarSetting& lidar_setting) {
  geometry_msgs::TransformStamped lidar_tf_msg;
  lidar_tf_msg.header.frame_id =
      vehicle_name + frame_name_base_link_;  // todo
                                             // multiple drones
  lidar_tf_msg.child_frame_id = lidar_name;
  lidar_tf_msg.transform.translation.x = lidar_setting.position.x();
  lidar_tf_msg.transform.translation.y = lidar_setting.position.y();
  lidar_tf_msg.transform.translation.z = lidar_setting.position.z();
  tf2::Quaternion quat;
  quat.setRPY(lidar_setting.rotation.roll,
              lidar_setting.rotation.pitch,
              lidar_setting.rotation.yaw);
  lidar_tf_msg.transform.rotation.x = quat.x();
  lidar_tf_msg.transform.rotation.y = quat.y();
  lidar_tf_msg.transform.rotation.z = quat.z();
  lidar_tf_msg.transform.rotation.w = quat.w();

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::Transform tf_BaseNED_LidarNED, tf_BaseNWU_LidarNWU;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);
    tf2::convert(lidar_tf_msg.transform, tf_BaseNED_LidarNED);

    tf_BaseNWU_LidarNWU = tf_nwu_ned * tf_BaseNED_LidarNED * tf_ned_nwu;
    tf2::convert(tf_BaseNWU_LidarNWU, lidar_tf_msg.transform);
  }

  static_tf_msg_vec_.push_back(lidar_tf_msg);
}

void AirsimROSWrapper::append_static_camera_tf(
    const std::string& vehicle_name,
    const std::string& camera_name,
    const CameraSetting& camera_setting) {
  geometry_msgs::TransformStamped static_cam_tf_body_msg;
  static_cam_tf_body_msg.header.frame_id = vehicle_name + frame_name_base_link_;
  static_cam_tf_body_msg.child_frame_id = camera_name + "_body/static";
  static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
  static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
  static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
  tf2::Quaternion quat;
  quat.setRPY(camera_setting.rotation.roll,
              camera_setting.rotation.pitch,
              camera_setting.rotation.yaw);
  static_cam_tf_body_msg.transform.rotation.x = quat.x();
  static_cam_tf_body_msg.transform.rotation.y = quat.y();
  static_cam_tf_body_msg.transform.rotation.z = quat.z();
  static_cam_tf_body_msg.transform.rotation.w = quat.w();

  geometry_msgs::TransformStamped static_cam_tf_optical_msg =
      static_cam_tf_body_msg;
  static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";

  tf2::Quaternion quat_cam_body;
  tf2::Quaternion quat_cam_optical;
  tf2::convert(static_cam_tf_body_msg.transform.rotation, quat_cam_body);
  tf2::Matrix3x3 mat_cam_body(quat_cam_body);
  tf2::Matrix3x3 mat_cam_optical;
  mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(),
                           mat_cam_body.getColumn(2).getX(),
                           mat_cam_body.getColumn(0).getX(),
                           mat_cam_body.getColumn(1).getY(),
                           mat_cam_body.getColumn(2).getY(),
                           mat_cam_body.getColumn(0).getY(),
                           mat_cam_body.getColumn(1).getZ(),
                           mat_cam_body.getColumn(2).getZ(),
                           mat_cam_body.getColumn(0).getZ());
  mat_cam_optical.getRotation(quat_cam_optical);
  quat_cam_optical.normalize();
  tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);

    // Handle static_cam_tf_body_msg (transform from the camera
    // body frame which is in NED that is rotate 180deg around x
    // to the vehicle's origin frame (similar to ROS /odom
    // frame))
    tf2::Transform tf_OdomNED_CamNED, tf_OdomNWU_CamNWU;
    tf2::convert(static_cam_tf_body_msg.transform, tf_OdomNED_CamNED);
    tf_OdomNWU_CamNWU = tf_nwu_ned * tf_OdomNED_CamNED * tf_ned_nwu;
    tf2::convert(tf_OdomNWU_CamNWU, static_cam_tf_body_msg.transform);

    // Handle static_cam_tf_optical_msg (transform from the
    // camera optical frame to the vehicle's origin frame
    // (similar to ROS /odom frame))
    tf2::Transform tf_OdomNED_CamOPT, tf_OdomNWU_CamOPT;
    tf2::convert(static_cam_tf_optical_msg.transform, tf_OdomNED_CamOPT);
    tf_OdomNWU_CamOPT = tf_nwu_ned * tf_OdomNED_CamOPT;
    tf2::convert(tf_OdomNWU_CamOPT, static_cam_tf_optical_msg.transform);
  }

  static_tf_msg_vec_.push_back(static_cam_tf_body_msg);
  static_tf_msg_vec_.push_back(static_cam_tf_optical_msg);
}

void AirsimROSWrapper::img_response_timer_cb(const ros::TimerEvent& event) {
  try {
    int image_response_idx = 0;
    for (const auto& airsim_img_request_vehicle_name_pair :
         airsim_img_request_vehicle_name_pair_vec_) {
      switch (vehicle_type_) {
        case CAR: {
          const std::vector<ImageResponse>& img_response =
              airsim_car_client_images_.simGetImages(
                  airsim_img_request_vehicle_name_pair.first,
                  airsim_img_request_vehicle_name_pair.second);
          if (img_response.size() ==
              airsim_img_request_vehicle_name_pair.first.size()) {
            process_and_publish_img_response(
                img_response,
                image_response_idx,
                airsim_img_request_vehicle_name_pair.second);
            image_response_idx += img_response.size();
          }
          break;
        }
        case MULTIROTOR: {
          const std::vector<ImageResponse>& img_response =
              airsim_client_images_.simGetImages(
                  airsim_img_request_vehicle_name_pair.first,
                  airsim_img_request_vehicle_name_pair.second);
          if (img_response.size() ==
              airsim_img_request_vehicle_name_pair.first.size()) {
            process_and_publish_img_response(
                img_response,
                image_response_idx,
                airsim_img_request_vehicle_name_pair.second);
            image_response_idx += img_response.size();
          }
          break;
        }
        default:
          LOG(FATAL) << "Unknown vehicle type";
      }
    }
  }

  catch (rpc::rpc_error& e) {
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API, didn't get "
                 "image response."
              << std::endl
              << msg << std::endl;
  }
}

void AirsimROSWrapper::lidar_timer_cb(const ros::TimerEvent& event) {
  try {
    if (lidar_pub_vec_.size() > 0) {
      int ctr = 0;
      for (const auto& vehicle_lidar_pair : vehicle_lidar_map_) {
        sensor_msgs::PointCloud2 lidar_msg;
        switch (vehicle_type_) {
          case CAR: {
            auto lidar_data = airsim_car_client_lidar_.getLidarData(
                vehicle_lidar_pair.second, vehicle_lidar_pair.first);
            // airsim api is imu_name, vehicle_name

            // todo make const ptr msg to avoid copy
            lidar_msg = get_lidar_msg_from_airsim(lidar_data);
            break;
          }
          case MULTIROTOR: {
            auto lidar_data = airsim_client_lidar_.getLidarData(
                vehicle_lidar_pair.second, vehicle_lidar_pair.first);
            // todo make const ptr msg to avoid copy
            lidar_msg = get_lidar_msg_from_airsim(lidar_data);
            break;
          }
          default:
            LOG(FATAL) << "Unknown vehicle type";
        }

        // sensor frame name. todo add to doc
        lidar_msg.header.frame_id = vehicle_lidar_pair.second;
        lidar_msg.header.stamp = ros::Time::now();

        // Transform to NWU
        if (use_nwu_std_) {
          ros::Time msg_time = lidar_msg.header.stamp;
          tf2::doTransform(lidar_msg, lidar_msg, trans_nwu_ned_);
          lidar_msg.header.frame_id = vehicle_lidar_pair.second;
          lidar_msg.header.stamp = msg_time;
        }

        lidar_pub_vec_[ctr].publish(lidar_msg);
        ctr++;
      }
    }

  }

  catch (rpc::rpc_error& e) {
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API, didn't get "
                 "image response."
              << std::endl
              << msg << std::endl;
  }
}

cv::Mat AirsimROSWrapper::manual_decode_depth(
    const ImageResponse& img_response) const {
  cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
  int img_width = img_response.width;

  for (int row = 0; row < img_response.height; row++)
    for (int col = 0; col < img_width; col++)
      mat.at<float>(row, col) =
          img_response.image_data_float[row * img_width + col];
  return mat;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_img_msg_from_response(
    const ImageResponse& img_response,
    const ros::Time curr_ros_time,
    const std::string frame_id,
    const bool& use_img_response_time) {
  sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
  img_msg_ptr->data = img_response.image_data_uint8;

  int channel_num = static_cast<int>(img_response.image_data_uint8.size()) /
                    (img_response.width * img_response.height);
  img_msg_ptr->step = img_response.width * channel_num;

  if (use_img_response_time) {
    img_msg_ptr->header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
  } else {
    img_msg_ptr->header.stamp = curr_ros_time;
  }
  img_msg_ptr->header.frame_id = frame_id;
  img_msg_ptr->height = img_response.height;
  img_msg_ptr->width = img_response.width;

  if (channel_num == 3) {
    img_msg_ptr->encoding = "bgr8";
  } else if (channel_num == 4) {
    img_msg_ptr->encoding = "bgra8";
  } else {
    LOG(WARNING) << "Unexpected channel num " << channel_num
                 << " for received image.";
  }

  if (is_vulkan_) {
    if (channel_num == 3) {
      img_msg_ptr->encoding = "rgb8";
    } else if (channel_num == 4) {
      img_msg_ptr->encoding = "rgba8";
    } else {
      LOG(WARNING) << "Unexpected channel num " << channel_num
                   << " for received image.";
    }
  }
  img_msg_ptr->is_bigendian = 0;
  return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_depth_img_msg_from_response(
    const ImageResponse& img_response,
    const ros::Time curr_ros_time,
    const std::string frame_id,
    const bool& use_img_response_time) {
  // todo using img_response.image_data_float direclty as done
  // get_img_msg_from_response() throws an error, hence the
  // dependency on opencv and cv_bridge. however, this is an
  // extremely fast op, so no big deal.
  cv::Mat depth_img = manual_decode_depth(img_response);
  sensor_msgs::ImagePtr depth_img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
  if (use_img_response_time) {
    depth_img_msg->header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
  } else {
    depth_img_msg->header.stamp = curr_ros_time;
  }
  depth_img_msg->header.frame_id = frame_id;
  return depth_img_msg;
}

// todo have a special stereo pair mode and get projection
// matrix by calculating offset wrt drone body frame?
sensor_msgs::CameraInfo AirsimROSWrapper::generate_cam_info(
    const std::string& vehicle_name,
    const std::string& camera_name,
    const CameraSetting& camera_setting,
    const CaptureSetting& capture_setting) const {
  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = camera_name + "_optical";
  cam_info_msg.height = capture_setting.height;
  cam_info_msg.width = capture_setting.width;
  float f_x = (capture_setting.width / 2.0) /
              tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
  // todo focal length in Y direction should be same as X it
  // seems. this can change in future a scene capture component
  // which exactly correponds to a cine camera float f_y =
  // (capture_setting.height / 2.0) /
  // tan(math_common::deg2rad(fov_degrees / 2.0));
  cam_info_msg.K = {f_x,
                    0.0,
                    capture_setting.width / 2.0,
                    0.0,
                    f_x,
                    capture_setting.height / 2.0,
                    0.0,
                    0.0,
                    1.0};

  // TODO: If the stereo pair are not setup perfectly aligned,
  // the rectification matrix and the projection matrix need to
  // be loaded from a calibration file
  cam_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  // Calculate the projection matrix for the right camera given
  // the base line and f_x values
  std::cout << "vehicle_name: " << vehicle_name << std::endl;
  auto it = vehicle_name_stereo_baseline_map_.find(vehicle_name);
  if (camera_name == "StereoRight" &&
      it != vehicle_name_stereo_baseline_map_.end()) {
    float stereo_baseline = it->second;
    cam_info_msg.P = {f_x,
                      0.0,
                      capture_setting.width / 2.0,
                      -f_x * stereo_baseline,
                      0.0,
                      f_x,
                      capture_setting.height / 2.0,
                      0.0,
                      0.0,
                      0.0,
                      1.0,
                      0.0};
    return cam_info_msg;
  } else {
    cam_info_msg.P = {f_x,
                      0.0,
                      capture_setting.width / 2.0,
                      0.0,
                      0.0,
                      f_x,
                      capture_setting.height / 2.0,
                      0.0,
                      0.0,
                      0.0,
                      1.0,
                      0.0};
    return cam_info_msg;
  }
}

void AirsimROSWrapper::process_and_publish_img_response(
    const std::vector<ImageResponse>& img_response_vec,
    const int img_response_idx,
    const std::string& vehicle_name) {
  // todo add option to use airsim time
  // (image_response.TTimePoint) like Gazebo /use_sim_time param
  ros::Time curr_ros_time;
  int img_response_idx_internal = img_response_idx;
  if (kForceSyncImages_) {
    curr_ros_time =
        airsim_timestamp_to_ros(img_response_vec.front().time_stamp);
  } else {
    curr_ros_time = ros::Time::now();
  }

  for (const auto& curr_img_response : img_response_vec) {
    // if a render request failed for whatever reason, this img
    // will be empty. Attempting to use a make_ts(0) results in
    // ros::Duration runtime error.
    if (curr_img_response.time_stamp == 0) continue;

    // TODO(srabiee): Current naming of frame ID's can lead to
    // duplicates if multiple vehicles use the same sensor
    // names. It is better to append sensor names with the name
    // of the vehicle

    // todo publishing a tf for each capture type seems stupid.
    // but it foolproofs us against render thread's async stuff,
    // I hope. Ideally, we should loop over cameras and then
    // captures, and publish only one tf.
    publish_camera_tf(curr_img_response,
                      curr_ros_time,
                      vehicle_name,
                      curr_img_response.camera_name,
                      !kForceSyncImages_);

    // todo simGetCameraInfo is wrong + also it's only for image
    // type -1. msr::airlib::CameraInfo camera_info =
    // airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

    // update timestamp of saved cam info msgs
    if (kForceSyncImages_) {
      camera_info_msg_vec_[img_response_idx_internal].header.stamp =
          curr_ros_time;
    } else {
      camera_info_msg_vec_[img_response_idx_internal].header.stamp =
          airsim_timestamp_to_ros(curr_img_response.time_stamp);
    }

    cam_info_pub_vec_[img_response_idx_internal].publish(
        camera_info_msg_vec_[img_response_idx_internal]);

    // DepthPlanner / DepthPerspective / DepthVis /
    // DisparityNormalized
    if (curr_img_response.pixels_as_float) {
      image_pub_vec_[img_response_idx_internal].publish(
          get_depth_img_msg_from_response(
              curr_img_response,
              curr_ros_time,
              curr_img_response.camera_name + "_optical",
              !kForceSyncImages_));
    }
    // Scene / Segmentation / SurfaceNormals / Infrared
    else {
      image_pub_vec_[img_response_idx_internal].publish(
          get_img_msg_from_response(curr_img_response,
                                    curr_ros_time,
                                    curr_img_response.camera_name + "_optical",
                                    !kForceSyncImages_));
    }
    img_response_idx_internal++;
  }
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are
// in (local) NED frame. We first do a change of basis to camera
// optical frame (Z forward, X right, Y down)
void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response,
                                         const ros::Time& ros_time,
                                         const std::string& frame_id,
                                         const std::string& child_frame_id,
                                         const bool& use_img_response_time) {
  geometry_msgs::TransformStamped cam_tf_body_msg;
  if (use_img_response_time) {
    cam_tf_body_msg.header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
  } else {
    cam_tf_body_msg.header.stamp = ros_time;
  }
  cam_tf_body_msg.header.frame_id = frame_id;
  cam_tf_body_msg.child_frame_id = child_frame_id + "_body";
  cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
  cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
  cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
  cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
  cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
  cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
  cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

  geometry_msgs::TransformStamped cam_tf_optical_msg;
  if (use_img_response_time) {
    cam_tf_optical_msg.header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
  } else {
    cam_tf_optical_msg.header.stamp = ros_time;
  }
  cam_tf_optical_msg.header.stamp = ros_time;
  cam_tf_optical_msg.header.frame_id = frame_id;
  cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
  cam_tf_optical_msg.transform.translation.x =
      cam_tf_body_msg.transform.translation.x;
  cam_tf_optical_msg.transform.translation.y =
      cam_tf_body_msg.transform.translation.y;
  cam_tf_optical_msg.transform.translation.z =
      cam_tf_body_msg.transform.translation.z;

  tf2::Quaternion quat_cam_body;
  tf2::Quaternion quat_cam_optical;
  tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
  tf2::Matrix3x3 mat_cam_body(quat_cam_body);
  // tf2::Matrix3x3 mat_cam_optical =
  // matrix_cam_body_to_optical_ * mat_cam_body
  // * matrix_cam_body_to_optical_inverse_; tf2::Matrix3x3
  // mat_cam_optical = matrix_cam_body_to_optical_ *
  // mat_cam_body;
  tf2::Matrix3x3 mat_cam_optical;
  mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(),
                           mat_cam_body.getColumn(2).getX(),
                           mat_cam_body.getColumn(0).getX(),
                           mat_cam_body.getColumn(1).getY(),
                           mat_cam_body.getColumn(2).getY(),
                           mat_cam_body.getColumn(0).getY(),
                           mat_cam_body.getColumn(1).getZ(),
                           mat_cam_body.getColumn(2).getZ(),
                           mat_cam_body.getColumn(0).getZ());
  mat_cam_optical.getRotation(quat_cam_optical);
  quat_cam_optical.normalize();
  tf2::convert(quat_cam_optical, cam_tf_optical_msg.transform.rotation);

  if (use_nwu_std_) {
    tf2::Transform tf_ned_nwu, tf_nwu_ned;
    tf2::convert(trans_ned_nwu_.transform, tf_ned_nwu);
    tf2::convert(trans_nwu_ned_.transform, tf_nwu_ned);

    // Handle cam_tf_body_msg (transform from the camera body
    // frame which is in NED to the vehicle's origin frame
    // (similar to ROS /odom frame))
    tf2::Transform tf_OdomNED_CamNED, tf_OdomNWU_CamNWU;
    tf2::convert(cam_tf_body_msg.transform, tf_OdomNED_CamNED);
    tf_OdomNWU_CamNWU = tf_nwu_ned * tf_OdomNED_CamNED * tf_ned_nwu;
    tf2::convert(tf_OdomNWU_CamNWU, cam_tf_body_msg.transform);

    // Handle cam_tf_optical_msg (transform from the camera
    // optical frame to the vehicle's origin frame (similar to
    // ROS /odom frame))
    tf2::Transform tf_OdomNED_CamOPT, tf_OdomNWU_CamOPT;
    tf2::convert(cam_tf_optical_msg.transform, tf_OdomNED_CamOPT);
    tf_OdomNWU_CamOPT = tf_nwu_ned * tf_OdomNED_CamOPT;
    tf2::convert(tf_OdomNWU_CamOPT, cam_tf_optical_msg.transform);
  }

  tf_broadcaster_.sendTransform(cam_tf_body_msg);
  tf_broadcaster_.sendTransform(cam_tf_optical_msg);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node,
                                                  SimpleMatrix& m) const {
  int rows, cols;
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();
  const YAML::Node& data = node["data"];
  for (int i = 0; i < rows * cols; ++i) {
    m.data[i] = data[i].as<double>();
  }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(
    const std::string& file_name, sensor_msgs::CameraInfo& cam_info) const {
  std::ifstream fin(file_name.c_str());
  YAML::Node doc = YAML::Load(fin);

  cam_info.width = doc[WIDTH_YML_NAME].as<int>();
  cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

  SimpleMatrix K_(3, 3, &cam_info.K[0]);
  convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
  SimpleMatrix R_(3, 3, &cam_info.R[0]);
  convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
  SimpleMatrix P_(3, 4, &cam_info.P[0]);
  convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

  cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

  const YAML::Node& D_node = doc[D_YML_NAME];
  int D_rows, D_cols;
  D_rows = D_node["rows"].as<int>();
  D_cols = D_node["cols"].as<int>();
  const YAML::Node& D_data = D_node["data"];
  cam_info.D.resize(D_rows * D_cols);
  for (int i = 0; i < D_rows * D_cols; ++i) {
    cam_info.D[i] = D_data[i].as<float>();
  }
}
