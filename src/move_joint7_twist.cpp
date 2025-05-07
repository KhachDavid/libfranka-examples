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

    constexpr double omega   = 0.1;      // rad/s about tool-Z
    constexpr double T = 2.0;            // s
    double t = 0.0;

    robot.control([&](const franka::RobotState&, franka::Duration dt)
                  -> franka::CartesianVelocities {
      t += dt.toSec();
      std::array<double, 6> twist{};          // vx vy vz wx wy wz
      twist[5] = (t < T) ? omega : 0.0;

      if (t >= T)
        return franka::MotionFinished(franka::CartesianVelocities(twist));
      return franka::CartesianVelocities(twist);
    });

  } catch (const franka::Exception& e) { logError(e); return -1; }
  return 0;
}
