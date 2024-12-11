#pragma once

namespace str {
class DataUtils {
 public:
  DataUtils() = delete;
  static void SetupDataLogging();
  static void LogGitInfo();
};
}  // namespace str