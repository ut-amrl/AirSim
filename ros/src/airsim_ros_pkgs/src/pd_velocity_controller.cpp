#include "airsim_ros_wrapper.h"

PIDVelocityController::PIDVelocityController() {
    target_velocity_ = 0.0;
    target_steering_ = 0.0;
}

void PIDVelocityController::set_target(const VelCmd& cmd) {
    target_velocity_ = cmd.x;
    target_steering_ = cmd.yaw_mode.yaw_or_rate;
    last_error_ = -DBL_MAX;
    last_integral_ = -DBL_MAX;
}

msr::airlib::CarApiBase::CarControls PIDVelocityController::get_next(const msr::airlib::Twist& current_twist, const double timestep) {
  msr::airlib::CarApiBase::CarControls controls;
  // TODO(Kavan): send control commands to car (use airsim_car_client_)
  double currentVel = current_twist.linear.x();
  double targetVel = target_velocity_;

  double acc;
  
  double error = targetVel - currentVel;
  if (last_integral_ != -DBL_MAX && last_error_ != -DBL_MAX) {
    last_integral_ += error*timestep;
    double derivative = (error - last_error_) / timestep;
    acc = PIDVelocityController::ERROR_WEIGHT * error + PIDVelocityController::INTEGRAL_WEIGHT * last_integral_ + PIDVelocityController::DERIV_WEIGHT * derivative;
    printf("ERROR %f\t DERIVATIVE %f\t INTEGRAL %f\t", error, derivative, last_integral_);
  } else {
    acc = PIDVelocityController::ERROR_WEIGHT * error;
    last_integral_ = error*timestep;
  }
  printf("COMPUTED ACC %f\n", acc);
  last_error_ = error;

  // For the moment, a really dumb controller, only forward/backward
  if (currentVel < targetVel - PIDVelocityController::VEL_EPSILON) {
      // printf("+: Velocities %f %f\n", currentVel, targetVel);
      controls.throttle = acc;
      controls.is_manual_gear = true;
      controls.manual_gear = 1;
  } else if (currentVel > targetVel + PIDVelocityController::VEL_EPSILON) {
      // printf("-: Velocities %f %f\n", currentVel, targetVel);
      controls.brake = std::abs(acc);
  } else {
      // Only turn off the vel cmd once we have achieved target velocity
      // TODO figure out how to maintain speed...
      // car_ros.has_vel_cmd = false;
  }

  // handle steering
  double currentYaw = current_twist.angular.z();
  double targetYaw = target_steering_;
  // printf("Yaws %f %f\n", currentYaw, targetYaw);
  // For the moment, a really dumb controller, only forward/backward
  if (currentYaw < targetYaw - PIDVelocityController::VEL_EPSILON) {
      controls.steering = (targetYaw - currentYaw) / targetYaw;
  } else if (currentYaw > targetYaw + PIDVelocityController::VEL_EPSILON) {
      controls.steering = (targetYaw - currentYaw) / targetYaw;
  } else {
      // Only turn off the vel cmd once we have achieved target velocity
      // car_ros.has_vel_cmd = false;
  }

  return controls;
}
