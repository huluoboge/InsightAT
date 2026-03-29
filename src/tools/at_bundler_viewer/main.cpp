/**
 * @file  main.cpp
 * @brief 独立可执行程序：可视化 Bundler 导出（list.txt + bundle.out）。
 */

#include "bundler_viewer_window.h"

#include <glog/logging.h>

#include <QApplication>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  QApplication app(argc, argv);
  insight::BundlerViewerWindow w;
  w.show();
  return app.exec();
}
