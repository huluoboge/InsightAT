#include "cli_logging.h"
#include <algorithm>
#include <cctype>
#include <glog/logging.h>
#include <iostream>

namespace insight {
namespace tools {

namespace {

int parse_level(const std::string& s) {
  std::string lower;
  lower.reserve(s.size());
  for (char c : s) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lower == "error")
    return 2;
  if (lower == "warn" || lower == "warning")
    return 1;
  if (lower == "info")
    return 0;
  if (lower == "debug")
    return 0; // minloglevel=INFO, FLAGS_v set separately
  return -1;
}

bool is_debug_level(const std::string& s) {
  std::string lower;
  lower.reserve(s.size());
  for (char c : s) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return lower == "debug";
}

} // namespace

void apply_log_level(bool verbose, bool quiet, const std::string& log_level) {
  if (!log_level.empty()) {
    int level = parse_level(log_level);
    if (level >= 0) {
      FLAGS_minloglevel = level;
      FLAGS_v = is_debug_level(log_level) ? 1 : 0;
      return;
    }
    std::cerr << "Warning: unknown --log-level='" << log_level
              << "', use error|warn|info|debug; defaulting to warn.\n";
  }

  if (quiet) {
    FLAGS_minloglevel = 2; // ERROR
    FLAGS_v = 0;
    return;
  }
  if (verbose) {
    FLAGS_minloglevel = 0; // INFO
    FLAGS_v = 0;
    return;
  }

  FLAGS_minloglevel = 1; // WARNING
  FLAGS_v = 0;
}

} // namespace tools
} // namespace insight
