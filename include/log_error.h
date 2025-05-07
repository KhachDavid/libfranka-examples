/* log_error.h ----------------------------------------------------------- */
#pragma once
#include <iostream>
#include <iomanip>
#include <typeinfo>

#include <franka/exception.h>      // defines all exception classes
#include <franka/log.h>            // brings in franka::Record

inline void logError(const franka::Exception& ex) {
  std::cerr << "\n===== libfranka exception =====\n";
  std::cerr << "Type : " << typeid(ex).name() << '\n';
  std::cerr << "What : " << ex.what() << '\n';

  /* Extra diagnostics for ControlException ----------------------------- */
  if (const auto* ce = dynamic_cast<const franka::ControlException*>(&ex)) {
    std::cerr << "Last " << ce->log.size() << " records kept by libfranka:\n";
    std::size_t shown = 0;
    for (const auto& rec : ce->log) {
      if (++shown > 10) {          // avoid spamming the console
        std::cerr << "  ... (" << ce->log.size() - 10
                  << " more records omitted)\n";
        break;
      }
      const auto& s = rec.state;   // direct member access
      std::cerr << "  t = " << std::fixed << std::setprecision(6)
                << s.time.toSec() << "  q7 = " << s.q[6] << '\n';
    }
  }
  std::cerr << "===============================\n";
}
