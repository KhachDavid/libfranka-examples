#include <iostream>
#include "log_error.h"

#include "examples_common.h"
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/control_types.h>
#include <Eigen/Dense>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: " << argv[0] << " <robot-ip>\n"; return -1; }

  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    const auto start_pose = robot.readOnce().O_T_EE_c;   // 4×4, col-major
    Eigen::Map<const Eigen::Matrix4d> T0(start_pose.data());
    Eigen::Matrix3d R0 = T0.block<3,3>(0,0);
    Eigen::Vector3d p  = T0.block<3,1>(0,3);

    constexpr double dtheta = 0.2;
    constexpr double T = 2.0;
    double t = 0.0;

    robot.control([&](const franka::RobotState&, franka::Duration dt)
                  -> franka::CartesianPose {
      t += dt.toSec();
      double a = std::min(1.0, t / T);

      Eigen::AngleAxisd Rz(a * dtheta, Eigen::Vector3d::UnitZ());
      Eigen::Matrix4d Tcmd = Eigen::Matrix4d::Identity();
      Tcmd.block<3,3>(0,0) = Rz * R0;
      Tcmd.block<3,1>(0,3) = p;

      std::array<double, 16> pose{};
      Eigen::Map<Eigen::Matrix<double,4,4,Eigen::ColMajor>>(pose.data()) = Tcmd;

      if (a >= 1.0)
        return franka::MotionFinished(franka::CartesianPose(pose));
      return franka::CartesianPose(pose);
    });

  } catch (const franka::Exception& e) { logError(e); return -1; }
  return 0;
}
