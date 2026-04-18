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
#include <QRegularExpression>
#include <QString>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  QApplication app(argc, argv);
  insight::BundlerViewerWindow w;
  w.show();

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
      w.open_iter_series_from_path(path);
    else
      w.open_reconstruction_path(path);
  }

  return app.exec();
}
