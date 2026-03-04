#ifndef INSIGHTAT_TOOLS_CLI_LOGGING_H
#define INSIGHTAT_TOOLS_CLI_LOGGING_H

#include <string>

namespace insight {
namespace tools {

/// Applies unified log level to glog from CLI flags.
/// Priority: log_level (if non-empty) > quiet > verbose > default "warn".
///
/// \param verbose  True if -v/--verbose was used
/// \param quiet    True if -q/--quiet was used
/// \param log_level  Value of --log-level (error|warn|info|debug), or empty
void apply_log_level(bool verbose, bool quiet, const std::string& log_level);

} // namespace tools
} // namespace insight

#endif // INSIGHTAT_TOOLS_CLI_LOGGING_H
