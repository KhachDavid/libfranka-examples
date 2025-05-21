#include <iostream>
#include "log_error.h"

#include "examples_common.h"
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/control_types.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> <torque>\n";
    return -1;
  }

  double max_torque = std::stod(argv[2]);
  if (max_torque < -10.0 || max_torque > 10.0) {
    std::cerr << "Torque must be between -10.0 and 10.0 Nm.\n";
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);  // Keeps torque rate saturation enabled

    constexpr double ramp_time = 2.0;   // Ramp over 1 second
    constexpr double total_duration = 2.0;

    double time = 0.0;

    robot.control([&](const franka::RobotState& state, franka::Duration dt) -> franka::Torques {
      time += dt.toSec();
      std::array<double, 7> tau{};

      // Linear ramp up
      double ramped_torque = (time < ramp_time) ? (max_torque * time / ramp_time) : max_torque;
      tau[6] = ramped_torque;

      // Stop after total duration
      if (time >= total_duration) {
        std::cout << "Finished torque command.\n";
        return franka::MotionFinished(franka::Torques(tau));
      }

      return franka::Torques(tau);
    });

  } catch (const franka::Exception& e) {
    logError(e);
    return -1;
  }

  return 0;
}
