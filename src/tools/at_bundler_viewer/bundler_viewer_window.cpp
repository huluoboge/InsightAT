/**
 * @file  bundler_viewer_window.cpp
 */

#include "bundler_viewer_window.h"

#include <algorithm>

#include "render/bundler_loader.h"
#include "render/render_tracks.h"
#include "render/render_widget.h"

#include <glog/logging.h>

#include <QAction>
#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace insight {

namespace {

QPushButton* make_toggle(const QString& text, const QString& tooltip, bool checked) {
  auto* b = new QPushButton(text);
  b->setCheckable(true);
  b->setChecked(checked);
  b->setToolTip(tooltip);
  return b;
}

QPushButton* make_push(const QString& text, const QString& tooltip) {
  auto* b = new QPushButton(text);
  b->setToolTip(tooltip);
  return b;
}

} // namespace

BundlerViewerWindow::BundlerViewerWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QStringLiteral("Bundler viewer"));
  resize(1024, 768);

  auto* central = new QWidget(this);
  setCentralWidget(central);
  auto* outer = new QVBoxLayout(central);
  outer->setContentsMargins(4, 4, 4, 4);
  outer->setSpacing(4);

  auto* bar = new QHBoxLayout();
  bar->setSpacing(6);

  // Reset view (was "Home")
  auto* btn_reset = make_push(
      tr("Reset view"),
      tr("Reset camera and scene transform to the default (same as “home” in the main viewer)."));
  connect(btn_reset, &QPushButton::clicked, this, &BundlerViewerWindow::on_reset_view);
  bar->addWidget(btn_reset);

  // Pivot ball
  auto* btn_pivot =
      make_toggle(tr("Pivot"),
                  tr("Show or hide the rotation pivot handle (the draggable ball)."), false);
  connect(btn_pivot, &QPushButton::toggled, this, &BundlerViewerWindow::on_pivot_toggled);
  bar->addWidget(btn_pivot);

  // Cameras (frustums)
  auto* btn_cams = make_toggle(
      tr("Cameras"),
      tr("Show or hide camera frustums (pose + approximate field of view)."), true);
  connect(btn_cams, &QPushButton::toggled, this, &BundlerViewerWindow::on_cameras_toggled);
  bar->addWidget(btn_cams);

  // 3D points
  auto* btn_pts = make_toggle(tr("Points"),
                              tr("Show or hide the sparse 3D point cloud."), true);
  connect(btn_pts, &QPushButton::toggled, this, &BundlerViewerWindow::on_points_toggled);
  bar->addWidget(btn_pts);

  bar->addSpacing(12);

  auto* btn_cam_minus = make_push(
      tr("Frustums −"),
      tr("Make camera frustum glyphs smaller (does not change the solved reconstruction)."));
  auto* btn_cam_plus = make_push(
      tr("Frustums +"),
      tr("Make camera frustum glyphs larger (visualization scale only)."));
  connect(btn_cam_minus, &QPushButton::clicked, this, &BundlerViewerWindow::on_frustum_smaller);
  connect(btn_cam_plus, &QPushButton::clicked, this, &BundlerViewerWindow::on_frustum_larger);
  bar->addWidget(btn_cam_minus);
  bar->addWidget(btn_cam_plus);

  bar->addSpacing(12);

  auto* btn_pt_minus =
      make_push(tr("Points −"), tr("Decrease 3D point display size (pixel diameter)."));
  auto* btn_pt_plus =
      make_push(tr("Points +"), tr("Increase 3D point display size (pixel diameter)."));
  connect(btn_pt_minus, &QPushButton::clicked, this, &BundlerViewerWindow::on_point_size_smaller);
  connect(btn_pt_plus, &QPushButton::clicked, this, &BundlerViewerWindow::on_point_size_larger);
  bar->addWidget(btn_pt_minus);
  bar->addWidget(btn_pt_plus);

  bar->addStretch(1);

  outer->addLayout(bar);

  render_widget_ = new render::RenderWidget(central);
  outer->addWidget(render_widget_, 1);

  tracks_ = new render::RenderTracks;
  render_widget_->data_root()->render_objects().push_back(tracks_);

  render_widget_->set_pivot_visible(btn_pivot->isChecked());

  auto* open_act = menuBar()->addMenu(tr("&File"))->addAction(tr("&Open Bundler folder…"));
  open_act->setShortcut(tr("Ctrl+O"));
  connect(open_act, &QAction::triggered, this, &BundlerViewerWindow::open_bundler_directory);
}

BundlerViewerWindow::~BundlerViewerWindow() = default;

void BundlerViewerWindow::open_bundler_directory() {
  const QString dir = QFileDialog::getExistingDirectory(this, tr("Bundler export folder"),
                                                        QString(), QFileDialog::ShowDirsOnly);
  if (dir.isEmpty())
    return;

  QProgressDialog progress(tr("Loading Bundler…"), QString(), 0, 0, this);
  progress.setWindowModality(Qt::ApplicationModal);
  progress.setMinimumDuration(0);
  progress.setCancelButton(nullptr);
  progress.setRange(0, 0);
  progress.setLabelText(tr("Reading bundle files…"));
  progress.show();
  QApplication::processEvents();

  render::BundlerScene scene;
  std::string err;
  if (!render::load_bundler_directory(dir.toStdString(), &scene, &err)) {
    progress.reset();
    QMessageBox::warning(this, tr("Load failed"), QString::fromStdString(err));
    LOG(ERROR) << "load_bundler_directory: " << err;
    return;
  }

  const int n_cam = static_cast<int>(scene.cameras.size());
  progress.setRange(0, std::max(1, n_cam));
  progress.setValue(0);
  progress.setLabelText(tr("Reading image dimensions…"));

  render::fill_render_tracks_from_bundler(
      tracks_, scene,
      [&](int current, int total, const char* /*stage*/) {
        progress.setValue(current);
        progress.setMaximum(std::max(1, total));
        progress.setLabelText(
            tr("Reading image dimensions… %1 / %2").arg(current).arg(total));
        QApplication::processEvents();
      });

  progress.setValue(progress.maximum());
  QApplication::processEvents();
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_reset_view() {
  render_widget_->fit_scene_to_view();
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_pivot_toggled(bool visible) {
  render_widget_->set_pivot_visible(visible);
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_cameras_toggled(bool visible) {
  tracks_->set_photo_visible(visible);
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_points_toggled(bool visible) {
  tracks_->set_vertex_visible(visible);
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_frustum_smaller() {
  tracks_->photo_smaller();
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_frustum_larger() {
  tracks_->photo_larger();
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_point_size_smaller() {
  tracks_->vertex_smaller();
  render_widget_->updateGL();
}

void BundlerViewerWindow::on_point_size_larger() {
  tracks_->vertex_large();
  render_widget_->updateGL();
}

} // namespace insight
