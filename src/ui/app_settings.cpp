/**
 * @file AppSettings.cpp
 * @brief Application-wide settings management implementation
 */

#include "app_settings.h"

#include <QSettings>
#include <QStandardPaths>
#include <QDir>
#include <glog/logging.h>

namespace insight {
namespace ui {

// Static member initialization
AppSettings* AppSettings::s_instance = nullptr;
const char* AppSettings::KEY_WORK_DIR = "work_directory";
const char* AppSettings::KEY_FIRST_LAUNCH = "first_launch";
const char* AppSettings::KEY_RECENT_PROJECTS = "recent_projects";

AppSettings::AppSettings() {
  // Constructor is private
}

AppSettings::~AppSettings() = default;

AppSettings& AppSettings::instance() {
  if (!s_instance) {
    s_instance = new AppSettings();
  }
  return *s_instance;
}

QString AppSettings::get_work_directory() const {
  QSettings settings("InsightAT", "InsightAT");
  return settings.value(KEY_WORK_DIR, "").toString();
}

void AppSettings::set_work_directory(const QString& path) {
  QSettings settings("InsightAT", "InsightAT");
  settings.setValue(KEY_WORK_DIR, path);
  settings.sync();
  LOG(INFO) << "Set work directory: " << path.toStdString();
}

bool AppSettings::has_work_directory() const {
  QString work_dir = get_work_directory();
  if (work_dir.isEmpty()) {
    return false;
  }
  QDir dir(work_dir);
  return dir.exists();
}

QStringList AppSettings::get_recent_projects() const {
  QSettings settings("InsightAT", "InsightAT");
  return settings.value(KEY_RECENT_PROJECTS, QStringList()).toStringList();
}

void AppSettings::add_recent_project(const QString& project_path) {
  QSettings settings("InsightAT", "InsightAT");
  QStringList recent = settings.value(KEY_RECENT_PROJECTS, QStringList()).toStringList();
  
  // Remove if already exists and add to front
  recent.removeAll(project_path);
  recent.prepend(project_path);
  
  // Keep only last 10 projects
  if (recent.size() > 10) {
    recent = recent.mid(0, 10);
  }
  
  settings.setValue(KEY_RECENT_PROJECTS, recent);
  settings.sync();
}

void AppSettings::clear_recent_projects() {
  QSettings settings("InsightAT", "InsightAT");
  settings.remove(KEY_RECENT_PROJECTS);
  settings.sync();
}

bool AppSettings::is_first_launch() const {
  QSettings settings("InsightAT", "InsightAT");
  return settings.value(KEY_FIRST_LAUNCH, true).toBool();
}

void AppSettings::mark_first_launch_done() {
  QSettings settings("InsightAT", "InsightAT");
  settings.setValue(KEY_FIRST_LAUNCH, false);
  settings.sync();
  LOG(INFO) << "First launch completed";
}

}  // namespace ui
}  // namespace insight
