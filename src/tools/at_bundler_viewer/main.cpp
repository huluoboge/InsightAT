/**
 * @file  main.cpp
 * @brief 独立可执行程序：可视化 Bundler 导出（list.txt + bundle.out）。
 *
 * Usage:
 *   at_bundler_viewer                        # open GUI, use File menu
 *   at_bundler_viewer <iter_series_dir>      # auto-load iteration series
 */

#include "bundler_viewer_window.h"

#include <glog/logging.h>

#include <QApplication>
#include <QString>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  QApplication app(argc, argv);
  insight::BundlerViewerWindow w;
  w.show();

  // If a directory is passed on the command line, open it as an iteration series.
  if (argc >= 2) {
    w.open_iter_series_from_path(QString::fromLocal8Bit(argv[1]));
  }

  return app.exec();
}
