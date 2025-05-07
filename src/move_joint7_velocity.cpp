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

    constexpr double dq        = 0.1;   // rad/s
    constexpr double run_time  = 2.0;   // s
    double time = 0.0;

    robot.control([&](const franka::RobotState&, franka::Duration dt)
                  -> franka::JointVelocities {
      time += dt.toSec();
      std::array<double, 7> dq_cmd{};
      dq_cmd[6] = (time < run_time) ? dq : 0.0;

      if (time >= run_time)
        return franka::MotionFinished(franka::JointVelocities(dq_cmd));
      return franka::JointVelocities(dq_cmd);
    });

  } catch (const franka::Exception& e) { logError(e); return -1; }
  return 0;
}
