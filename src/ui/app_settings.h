/**
 * @file AppSettings.h
 * @brief Application-wide settings management using Qt's QSettings
 *
 * Manages persistent application configuration including:
 * - Work directory (where projects are saved)
 * - Recent projects
 * - UI preferences
 */

#ifndef UI_APP_SETTINGS_H
#define UI_APP_SETTINGS_H

#include <QString>
#include <QStringList>

namespace insight {
namespace ui {

class AppSettings {
public:
  /**
   * Get singleton instance
   */
  static AppSettings& instance();

  /**
   * Get the configured work directory
   * @return Work directory path, empty if not set
   */
  QString get_work_directory() const;

  /**
   * Set the work directory
   * @param path Directory path
   */
  void set_work_directory(const QString& path);

  /**
   * Check if work directory is configured
   * @return true if work directory is set and exists
   */
  bool has_work_directory() const;

  /**
   * Get recent project files
   * @return List of recent project file paths
   */
  QStringList get_recent_projects() const;

  /**
   * Add a project to recent files list
   * @param project_path Full path to .iat file
   */
  void add_recent_project(const QString& project_path);

  /**
   * Clear recent projects
   */
  void clear_recent_projects();

  /**
   * Check if this is first launch
   * @return true if work directory has never been set
   */
  bool is_first_launch() const;

  /**
   * Mark first launch as completed
   */
  void mark_first_launch_done();

private:
  AppSettings();
  ~AppSettings();

  // Disable copy
  AppSettings(const AppSettings&) = delete;
  AppSettings& operator=(const AppSettings&) = delete;

  static AppSettings* s_instance;

  // Settings keys
  static const char* KEY_WORK_DIR;
  static const char* KEY_FIRST_LAUNCH;
  static const char* KEY_RECENT_PROJECTS;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_APP_SETTINGS_H
