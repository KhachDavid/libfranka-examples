#include <iostream>
#include "log_error.h"

#include "examples_common.h"
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/control_types.h>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: " << argv[0] << " <robot-ip>\n"; return -1; }

  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    const double q_start = robot.readOnce().q[6];
    const double q_des   = q_start + 0.2;

    constexpr double Kp = 25.0;   // Nm/rad
    constexpr double Kd = 3.0;    // Nm/(rad/s)
    double t = 0.0;

    robot.control([&](const franka::RobotState& s, franka::Duration dt)
                  -> franka::Torques {
      t += dt.toSec();
      std::array<double, 7> tau{};

      double e  = q_des - s.q[6];
      double de = -s.dq[6];
      tau[6] = Kp * e + Kd * de;

      if (t > 3.0 || std::abs(e) < 1e-3)
        return franka::MotionFinished(franka::Torques(tau));
      return franka::Torques(tau);
    });

  } catch (const franka::Exception& e) { logError(e); return -1; }
  return 0;
}
