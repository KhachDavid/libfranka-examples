#include <iostream>
#include <iomanip>
#include "log_error.h"

#include "examples_common.h"
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/control_types.h>

int main(int argc, char** argv) {
  if (argc < 3) {
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
    setDefaultBehavior(robot);

    constexpr double ramp_time = 2.0;         // ramp to torque over 2s
    constexpr double cutoff_time = 2.5;       // send 0 torque starting at this point
    constexpr double total_duration = 4.0;    // end after this much time

    double time = 0.0;

    robot.control([&](const franka::RobotState& state, franka::Duration dt) -> franka::Torques {
      time += dt.toSec();
      std::array<double, 7> tau{};

      if (time < cutoff_time) {
        // Ramp torque up to max_torque
        double ramped = (time < ramp_time) ? (max_torque * time / ramp_time) : max_torque;
        tau[6] = ramped;
      } else {
        // After cutoff_time, abruptly send 0 torque
        tau[6] = -0.1;
      }

      // Optional: Print debug info
      std::cout << std::fixed << std::setprecision(3)
                << "t=" << time << "  q[6]=" << state.q[6]
                << "  dq[6]=" << state.dq[6]
                << "  tau[6]=" << tau[6] << "\n";

      if (time >= total_duration) {
        std::cout << "Motion finished.\n";
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
