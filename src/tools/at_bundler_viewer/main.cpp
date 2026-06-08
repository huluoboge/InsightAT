/**
 * @file  main.cpp
 * @brief 独立可执行程序：可视化 Bundler 或 COLMAP text 稀疏模型。
 *
 * Usage:
 *   at_bundler_viewer                              # open GUI, use File menu
 *   at_bundler_viewer <dir>                        # auto-open dir
 *     — 若含 iter_NNNN 子目录：按增量迭代序列加载
 *     — 否则：单目录（Bundler：list.txt+bundle.out；或 COLMAP：cameras/images/points3D.txt）
 */

#include "bundler_viewer_window.h"

#include <glog/logging.h>

#include <QApplication>
#include <QDir>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QRegularExpression>
#include <QString>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  QApplication app(argc, argv);

  // Create main window for the standalone application
  auto* main_window = new QMainWindow();
  main_window->setWindowTitle(QObject::tr("InsightAT Bundler Viewer"));
  main_window->resize(1400, 900);

  // Create the viewer widget as central widget
  auto* viewer = new insight::BundlerViewerWindow(main_window);
  main_window->setCentralWidget(viewer);

  // Create File menu
  auto* file_menu = main_window->menuBar()->addMenu(QObject::tr("&File"));

  auto* open_act = file_menu->addAction(QObject::tr("&Open reconstruction folder…"));
  open_act->setShortcut(QObject::tr("Ctrl+O"));
  QObject::connect(open_act, &QAction::triggered, viewer, &insight::BundlerViewerWindow::open_bundler_directory);

  auto* open_series_act = file_menu->addAction(QObject::tr("Open &iteration series…"));
  open_series_act->setShortcut(QObject::tr("Ctrl+I"));
  QObject::connect(open_series_act, &QAction::triggered, viewer, &insight::BundlerViewerWindow::open_iter_series);

  file_menu->addSeparator();

  auto* exit_act = file_menu->addAction(QObject::tr("E&xit"));
  exit_act->setShortcut(QObject::tr("Ctrl+Q"));
  QObject::connect(exit_act, &QAction::triggered, main_window, &QMainWindow::close);

  main_window->show();

  if (argc >= 2) {
    const QString path = QString::fromLocal8Bit(argv[1]);
    QDir d(path);
    const QStringList entries = d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    const QRegularExpression iterRe(QStringLiteral("^iter_\\d+$"));
    bool has_iter_subdirs = false;
    for (const QString& e : entries) {
      if (iterRe.match(e).hasMatch()) {
        has_iter_subdirs = true;
        break;
      }
    }
    if (has_iter_subdirs)
      viewer->open_iter_series_from_path(path);
    else
      viewer->open_reconstruction_path(path);
  }

  return app.exec();
}
