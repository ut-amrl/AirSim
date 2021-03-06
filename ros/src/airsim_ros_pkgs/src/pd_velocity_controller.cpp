#include "airsim_ros_wrapper.h"

PIDVelocityController::PIDVelocityController() {
    target_velocity_ = 0.0;
    target_steering_ = 0.0;
}

void PIDVelocityController::set_target(const VelCmd& cmd) {
    target_velocity_ = cmd.x;
    target_steering_ = -cmd.yaw_mode.yaw_or_rate;
    last_error_ = -DBL_MAX;
    last_integral_ = -DBL_MAX;
}

void PIDVelocityController::set_zero_target() {
    target_velocity_ = 0;
    target_steering_ =0;
    last_error_ = -DBL_MAX;
    last_integral_ = -DBL_MAX;
}

msr::airlib::CarApiBase::CarControls PIDVelocityController::get_next(const msr::airlib::Twist& current_twist, float speed, const ros::Time curr_time) {
  msr::airlib::CarApiBase::CarControls controls;
  // TODO(Kavan): send control commands to car (use airsim_car_client_)
  double currentVel = speed;

  double targetVel = target_velocity_;

  double acc;
  double error = std::abs(targetVel - currentVel);
  double timestep = (curr_time - last_timestamp_).toSec();

  if (last_integral_ != -DBL_MAX) {
    last_integral_ += error*timestep;
    double derivative = (error - last_error_) / (timestep);
    acc = PIDVelocityController::K_p * error + PIDVelocityController::K_i * last_integral_ + PIDVelocityController::K_d * derivative;
    // printf("ERROR %f\t DERIVATIVE %f\t INTEGRAL %f Timestep %f\t", error, derivative, last_integral_, timestep);
  } else {
    acc = PIDVelocityController::K_p * error;
    last_integral_ = error*timestep;
  }

//   acc += MAINTENANCE_FACTOR * std::abs(targetVel);

//   printf("COMPUTED ACC %f\n", acc);
  last_error_ = error;
  last_timestamp_ = curr_time;
  
  // For the moment, a really dumb controller, only forward/backward
  if (currentVel < targetVel) {
    //   printf("+: Velocities %f %f\n", currentVel, targetVel);
      controls.throttle = std::min(acc, 1.0);
      controls.is_manual_gear = true;
      controls.manual_gear = 2;
  } else if (currentVel > targetVel + PIDVelocityController::VEL_BRAKE_THRESHOLD && currentVel > PIDVelocityController::VEL_EPSILON) {
    //   printf("-: Velocities %f %f\n", currentVel, targetVel);
      acc *= PIDVelocityController::BRAKING_SCALING_FACTOR;
      controls.brake = std::min(acc, 1.0);
  } else if (currentVel > targetVel && targetVel < 0) {
      // Reverse
      controls.throttle = std::min(acc, 1.0);
      controls.is_manual_gear = true;
      controls.manual_gear = -1;
  } else {
      // we are close but not quite, just coast
      controls.throttle = 0;
  }

  // handle steering
  double targetYaw = target_steering_;
  double currentYaw = current_twist.angular.z();

  // printf("Yaws %f %f\n", currentYaw, targetYaw);
  // For the moment, a really dumb controller, only forward/backward
  if (currentYaw < targetYaw - PIDVelocityController::VEL_EPSILON) {
      controls.steering = (targetYaw - currentYaw);
  } else if (currentYaw > targetYaw + PIDVelocityController::VEL_EPSILON) {
      controls.steering = (targetYaw - currentYaw);
  } else {
      // Only turn off the vel cmd once we have achieved target velocity
      // car_ros.has_vel_cmd = false;
  }

  return controls;
}
