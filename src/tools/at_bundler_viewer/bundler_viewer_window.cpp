/**
 * @file  bundler_viewer_window.cpp
 *
 * Performance model for iteration series:
 *   - First load (cold):  load_bundler_directory (text) + fill_render_tracks (GDAL per image).
 *     GDAL reads are cached globally in bundler_loader.cpp after the first call.
 *   - Warm load:          BundlerScene is in scene_cache_ (or prefetch future completed).
 *     fill_render_tracks is fast because image dimensions are already in the GDAL cache.
 *   - Typical result:     first iteration ~1–5 s; every subsequent iteration < 100 ms.
 */

#include "bundler_viewer_window.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "render/bundler_loader.h"
#include "render/render_tracks.h"
#include "render/render_widget.h"

#include <glog/logging.h>

#include <QAction>
#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QProgressDialog>
#include <QPushButton>
#include <QRegularExpression>
#include <QSplitter>
#include <QTextEdit>
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

/// RenderWidget subclass that emits a signal on left-click (for pick mode).
class PickableRenderWidget : public render::RenderWidget {
  Q_OBJECT
public:
  explicit PickableRenderWidget(QWidget* parent = nullptr) : render::RenderWidget(parent) {}

  void set_pick_mode(bool enabled) { pick_mode_ = enabled; }

signals:
  void clicked(int px, int py);

protected:
  void mousePressEvent(QMouseEvent* event) override {
    if (pick_mode_ && event->button() == Qt::LeftButton)
      m_press_pos = event->pos();
    render::RenderWidget::mousePressEvent(event);
  }
  void mouseReleaseEvent(QMouseEvent* event) override {
    if (pick_mode_ && event->button() == Qt::LeftButton) {
      if ((event->pos() - m_press_pos).manhattanLength() < 8)
        emit clicked(event->pos().x(), event->pos().y());
    }
    render::RenderWidget::mouseReleaseEvent(event);
  }

private:
  QPoint m_press_pos;
  bool   pick_mode_ = false;
};

// ── Construction ─────────────────────────────────────────────────────────────

BundlerViewerWindow::BundlerViewerWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QStringLiteral("Bundler viewer"));
  resize(1280, 800);

  auto* central = new QWidget(this);
  setCentralWidget(central);
  auto* outer = new QVBoxLayout(central);
  outer->setContentsMargins(4, 4, 4, 4);
  outer->setSpacing(4);

  // ── Main control bar ────────────────────────────────────────────────────────
  auto* bar = new QHBoxLayout();
  bar->setSpacing(6);

  auto* btn_reset = make_push(tr("Reset view"), tr("Reset camera and scene transform."));
  connect(btn_reset, &QPushButton::clicked, this, &BundlerViewerWindow::on_reset_view);
  bar->addWidget(btn_reset);

  auto* btn_pivot = make_toggle(tr("Pivot"), tr("Show / hide the rotation pivot handle."), false);
  connect(btn_pivot, &QPushButton::toggled, this, &BundlerViewerWindow::on_pivot_toggled);
  bar->addWidget(btn_pivot);

  auto* btn_cams = make_toggle(tr("Cameras"), tr("Show / hide camera frustums."), true);
  connect(btn_cams, &QPushButton::toggled, this, &BundlerViewerWindow::on_cameras_toggled);
  bar->addWidget(btn_cams);

  auto* btn_pts = make_toggle(tr("Points"), tr("Show / hide the sparse 3D point cloud."), true);
  connect(btn_pts, &QPushButton::toggled, this, &BundlerViewerWindow::on_points_toggled);
  bar->addWidget(btn_pts);

  bar->addSpacing(12);

  auto* btn_cam_minus = make_push(tr("Frustums −"), tr("Make camera frustum glyphs smaller."));
  auto* btn_cam_plus  = make_push(tr("Frustums +"), tr("Make camera frustum glyphs larger."));
  connect(btn_cam_minus, &QPushButton::clicked, this, &BundlerViewerWindow::on_frustum_smaller);
  connect(btn_cam_plus,  &QPushButton::clicked, this, &BundlerViewerWindow::on_frustum_larger);
  bar->addWidget(btn_cam_minus);
  bar->addWidget(btn_cam_plus);

  bar->addSpacing(12);

  auto* btn_pt_minus = make_push(tr("Points −"), tr("Decrease 3D point display size."));
  auto* btn_pt_plus  = make_push(tr("Points +"), tr("Increase 3D point display size."));
  connect(btn_pt_minus, &QPushButton::clicked, this, &BundlerViewerWindow::on_point_size_smaller);
  connect(btn_pt_plus,  &QPushButton::clicked, this, &BundlerViewerWindow::on_point_size_larger);
  bar->addWidget(btn_pt_minus);
  bar->addWidget(btn_pt_plus);

  bar->addSpacing(12);

  btn_pick_mode_ = make_toggle(tr("Pick"), tr("Enable pick mode: click a camera to see its name, click a 3D point to see its track observations."), false);
  connect(btn_pick_mode_, &QPushButton::toggled, this, &BundlerViewerWindow::on_pick_mode_toggled);
  bar->addWidget(btn_pick_mode_);

  bar->addStretch(1);
  outer->addLayout(bar);
  // ── Iteration navigation bar (hidden until a series is loaded) ─────────────
  iter_bar_widget_ = new QWidget(central);
  auto* iter_bar   = new QHBoxLayout(iter_bar_widget_);
  iter_bar->setContentsMargins(0, 0, 0, 0);
  iter_bar->setSpacing(6);

  btn_prev_iter_ = make_push(tr("◀  Prev iter"), tr("Previous iteration  (Left arrow)"));
  btn_next_iter_ = make_push(tr("Next iter  ▶"), tr("Next iteration  (Right arrow)"));
  connect(btn_prev_iter_, &QPushButton::clicked, this, &BundlerViewerWindow::on_prev_iter);
  connect(btn_next_iter_, &QPushButton::clicked, this, &BundlerViewerWindow::on_next_iter);
  iter_bar->addWidget(btn_prev_iter_);
  iter_bar->addWidget(btn_next_iter_);

  iter_slider_ = new QSlider(Qt::Horizontal);
  iter_slider_->setMinimum(0);
  iter_slider_->setMaximum(0);
  iter_slider_->setTickPosition(QSlider::TicksBelow);
  connect(iter_slider_, &QSlider::valueChanged, this,
          &BundlerViewerWindow::on_iter_slider_changed);
  iter_bar->addWidget(iter_slider_, 1);

  iter_label_ = new QLabel(tr("— / —"));
  iter_label_->setMinimumWidth(160);
  iter_label_->setAlignment(Qt::AlignCenter);
  iter_label_->setStyleSheet(QStringLiteral("font-weight: bold; font-family: monospace;"));
  iter_bar->addWidget(iter_label_);

  iter_bar_widget_->setVisible(false);
  outer->addWidget(iter_bar_widget_);

  // ── 3D view + pick info panel ────────────────────────────────────────────────
  auto* splitter = new QSplitter(Qt::Horizontal, central);

  auto* pickable = new PickableRenderWidget(splitter);
  render_widget_ = pickable;
  connect(pickable, &PickableRenderWidget::clicked,
          this, &BundlerViewerWindow::on_render_widget_clicked);
  splitter->addWidget(render_widget_);

  pick_info_panel_ = new QTextEdit(splitter);
  pick_info_panel_->setReadOnly(true);
  pick_info_panel_->setPlaceholderText(tr("Enable Pick mode and click a camera or 3D point."));
  pick_info_panel_->setMinimumWidth(220);
  pick_info_panel_->setMaximumWidth(400);
  pick_info_panel_->setVisible(false);
  splitter->addWidget(pick_info_panel_);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 0);

  outer->addWidget(splitter, 1);

  tracks_ = new render::RenderTracks;
  render_widget_->data_root()->render_objects().push_back(tracks_);
  render_widget_->set_pivot_visible(btn_pivot->isChecked());

  // ── Menu ────────────────────────────────────────────────────────────────────
  auto* file_menu = menuBar()->addMenu(tr("&File"));

  auto* open_act = file_menu->addAction(tr("&Open Bundler folder…"));
  open_act->setShortcut(tr("Ctrl+O"));
  connect(open_act, &QAction::triggered, this, &BundlerViewerWindow::open_bundler_directory);

  auto* open_series_act = file_menu->addAction(tr("Open &iteration series…"));
  open_series_act->setShortcut(tr("Ctrl+I"));
  connect(open_series_act, &QAction::triggered, this, &BundlerViewerWindow::open_iter_series);
}

BundlerViewerWindow::~BundlerViewerWindow() {
  cancel_prefetch();
}

// ── Single-directory open (original behaviour) ─────────────────────────────

void BundlerViewerWindow::open_bundler_directory() {
  const QString dir = QFileDialog::getExistingDirectory(
      this, tr("Bundler export folder"), QString(), QFileDialog::ShowDirsOnly);
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

  cancel_prefetch();
  iter_dirs_.clear();
  current_iter_idx_ = -1;
  dims_warmed_ = true;
  iter_bar_widget_->setVisible(false);

  // Keep a copy of the scene for pick queries
  current_scene_ = std::make_shared<render::BundlerScene>(std::move(scene));

  render_widget_->fit_scene_to_view();
  render_widget_->updateGL();
}

// ── Iteration-series open ───────────────────────────────────────────────────

void BundlerViewerWindow::open_iter_series() {
  const QString parent = QFileDialog::getExistingDirectory(
      this, tr("Select parent directory containing iter_NNNN folders"),
      QString(), QFileDialog::ShowDirsOnly);
  if (parent.isEmpty())
    return;
  open_iter_series_from_path(parent);
}

void BundlerViewerWindow::open_iter_series_from_path(const QString& parent_dir) {
  cancel_prefetch();
  {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    scene_cache_.clear();
  }
  dims_warmed_ = false;

  QDir dir(parent_dir);
  const QRegularExpression re(QStringLiteral("^iter_\\d+$"));
  QStringList entries = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

  iter_dirs_.clear();
  for (const QString& e : entries) {
    if (re.match(e).hasMatch())
      iter_dirs_.push_back(dir.absoluteFilePath(e).toStdString());
  }

  if (iter_dirs_.empty()) {
    QMessageBox::warning(this, tr("No iterations found"),
                         tr("No iter_NNNN subdirectories found in:\n%1").arg(parent_dir));
    return;
  }

  LOG(INFO) << "Iteration series: found " << iter_dirs_.size()
            << " iter dirs in " << parent_dir.toStdString();

  const int n = static_cast<int>(iter_dirs_.size());
  iter_slider_->setMinimum(0);
  iter_slider_->setMaximum(n - 1);
  iter_slider_->setTickInterval(std::max(1, n / 20));
  iter_bar_widget_->setVisible(true);

  load_iter_at(0);
}

// ── Scene cache and prefetch ────────────────────────────────────────────────

std::shared_ptr<render::BundlerScene>
BundlerViewerWindow::get_or_load_scene(int idx, bool* from_cache) {
  // 1. Check the scene cache first.
  {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    auto it = scene_cache_.find(idx);
    if (it != scene_cache_.end()) {
      *from_cache = true;
      return it->second;
    }
  }

  // 2. Check if a prefetch future exists for this index (ready or in-flight).
  std::future<std::shared_ptr<render::BundlerScene>> fut;
  {
    std::lock_guard<std::mutex> lk(prefetch_mutex_);
    auto it = prefetch_futures_.find(idx);
    if (it != prefetch_futures_.end()) {
      fut = std::move(it->second);
      prefetch_futures_.erase(it);
    }
  }
  if (fut.valid()) {
    // Wait for prefetch (it may already be done, or close to done).
    auto scene = fut.get();
    if (scene) {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      scene_cache_[idx] = scene;
      *from_cache = true; // was prefetched – treat as "no cold load"
      return scene;
    }
  }

  // 3. Synchronous load from disk.
  *from_cache = false;
  auto scene   = std::make_shared<render::BundlerScene>();
  std::string err;
  if (!render::load_bundler_directory(iter_dirs_[static_cast<size_t>(idx)], scene.get(), &err)) {
    LOG(ERROR) << "load_bundler_directory[" << idx << "]: " << err;
    return {};
  }
  {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    scene_cache_[idx] = scene;
  }
  return scene;
}

void BundlerViewerWindow::start_prefetch(int center_idx) {
  constexpr int kRadius = 3;
  const int n = static_cast<int>(iter_dirs_.size());

  for (int delta : {1, -1, 2, -2, 3, -3}) {
    const int target = center_idx + delta;
    if (target < 0 || target >= n)
      continue;

    // Skip if already cached.
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      if (scene_cache_.count(target))
        continue;
    }
    // Skip if already in-flight.
    {
      std::lock_guard<std::mutex> lk(prefetch_mutex_);
      if (prefetch_futures_.count(target))
        continue;
    }

    const std::string path = iter_dirs_[static_cast<size_t>(target)];
    auto fut = std::async(std::launch::async, [path]() {
      auto s = std::make_shared<render::BundlerScene>();
      std::string e;
      if (!render::load_bundler_directory(path, s.get(), &e)) {
        VLOG(1) << "Prefetch failed for " << path << ": " << e;
        return std::shared_ptr<render::BundlerScene>{};
      }
      VLOG(1) << "Prefetch ready: " << path;
      return s;
    });

    std::lock_guard<std::mutex> lk(prefetch_mutex_);
    prefetch_futures_[target] = std::move(fut);
  }
  (void)kRadius; // suppress unused warning
}

void BundlerViewerWindow::cancel_prefetch() {
  // Move all futures out (their destructors will block until threads finish).
  std::unordered_map<int, std::future<std::shared_ptr<render::BundlerScene>>> dying;
  {
    std::lock_guard<std::mutex> lk(prefetch_mutex_);
    dying = std::move(prefetch_futures_);
  }
  // Futures are joined here as 'dying' goes out of scope.
}

// ── Core loader for one iteration ──────────────────────────────────────────

void BundlerViewerWindow::load_iter_at(int idx) {
  if (idx < 0 || idx >= static_cast<int>(iter_dirs_.size()))
    return;

  bool from_cache = false;
  auto scene = get_or_load_scene(idx, &from_cache);
  if (!scene)
    return; // error already logged

  // Show progress dialog only on cold load (first time, GDAL dimension reads are slow).
  // After dims_warmed_=true all subsequent calls to fill_render_tracks are O(1) on GDAL.
  const bool need_progress = !dims_warmed_;

  if (need_progress) {
    const int n_cam = static_cast<int>(scene->cameras.size());
    QProgressDialog progress(tr("Loading first iteration…"), QString(), 0, std::max(1, n_cam),
                             this);
    progress.setWindowModality(Qt::ApplicationModal);
    progress.setMinimumDuration(0);
    progress.setCancelButton(nullptr);
    progress.setLabelText(tr("Reading image dimensions (first load)…"));
    progress.show();
    QApplication::processEvents();

    render::fill_render_tracks_from_bundler(
        tracks_, *scene,
        [&](int current, int total, const char* /*stage*/) {
          progress.setValue(current);
          progress.setMaximum(std::max(1, total));
          progress.setLabelText(
              tr("Reading image dimensions… %1 / %2").arg(current).arg(total));
          QApplication::processEvents();
        });

    progress.setValue(progress.maximum());
    QApplication::processEvents();
  } else {
    // Fast path: GDAL dimension cache is warm, fill is near-instant.
    render::fill_render_tracks_from_bundler(tracks_, *scene);
  }

  // Fit scene to view only on the very first load; preserve camera position on navigation.
  const bool do_fit = !dims_warmed_;
  dims_warmed_ = true;
  current_iter_idx_ = idx;
  current_scene_ = scene;
  tracks_->clear_selection();
  update_iter_controls();

  if (do_fit)
    render_widget_->fit_scene_to_view();
  render_widget_->updateGL();

  // Start prefetching adjacent iterations in the background.
  start_prefetch(idx);
}

void BundlerViewerWindow::update_iter_controls() {
  const int n   = static_cast<int>(iter_dirs_.size());
  const int idx = current_iter_idx_;

  QString folder_name;
  if (idx >= 0 && idx < n) {
    const std::string& p = iter_dirs_[static_cast<size_t>(idx)];
    const auto slash = p.rfind('/');
    folder_name = QString::fromStdString(slash == std::string::npos ? p : p.substr(slash + 1));
  }
  iter_label_->setText(tr("%1  (%2 / %3)").arg(folder_name).arg(idx + 1).arg(n));

  iter_slider_->blockSignals(true);
  iter_slider_->setValue(idx);
  iter_slider_->blockSignals(false);

  btn_prev_iter_->setEnabled(idx > 0);
  btn_next_iter_->setEnabled(idx < n - 1);

  setWindowTitle(tr("Bundler viewer — %1  [%2 / %3]").arg(folder_name).arg(idx + 1).arg(n));
}

// ── Navigation slots ────────────────────────────────────────────────────────

void BundlerViewerWindow::on_prev_iter() {
  if (current_iter_idx_ > 0)
    load_iter_at(current_iter_idx_ - 1);
}

void BundlerViewerWindow::on_next_iter() {
  if (current_iter_idx_ < static_cast<int>(iter_dirs_.size()) - 1)
    load_iter_at(current_iter_idx_ + 1);
}

void BundlerViewerWindow::on_iter_slider_changed(int value) {
  if (value != current_iter_idx_)
    load_iter_at(value);
}

// ── Keyboard shortcuts ──────────────────────────────────────────────────────

void BundlerViewerWindow::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape && pick_mode_active_) {
    btn_pick_mode_->setChecked(false); // triggers on_pick_mode_toggled(false)
    return;
  }
  if (!iter_dirs_.empty()) {
    if (event->key() == Qt::Key_Left) {
      on_prev_iter();
      return;
    }
    if (event->key() == Qt::Key_Right) {
      on_next_iter();
      return;
    }
  }
  QMainWindow::keyPressEvent(event);
}

// ── Standard control slots ──────────────────────────────────────────────────

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

// ── Pick mode ───────────────────────────────────────────────────────────────

void BundlerViewerWindow::on_pick_mode_toggled(bool enabled) {
  pick_mode_active_ = enabled;
  static_cast<PickableRenderWidget*>(render_widget_)->set_pick_mode(enabled);
  pick_info_panel_->setVisible(enabled);
  if (enabled) {
    pick_info_panel_->setPlainText(tr("Click a camera frustum or 3D point to inspect it."));
  } else {
    tracks_->clear_selection();
    render_widget_->updateGL();
  }
}

void BundlerViewerWindow::show_pick_info(const QString& text) {
  pick_info_panel_->setPlainText(text);
}

void BundlerViewerWindow::on_render_widget_clicked(int px, int py) {
  if (!pick_mode_active_ || !current_scene_)
    return;

  const render::RenderTracks::Photos& photos    = tracks_->photos();
  const render::RenderTracks::Tracks& track_list = tracks_->tracks();

  bool is_camera = false;
  int idx = tracks_->pick_screen(px, py, &is_camera);

  if (idx < 0) {
    tracks_->clear_selection();
    render_widget_->updateGL();
    show_pick_info(tr("Nothing picked. Try clicking closer to a camera or point."));
    return;
  }

  if (is_camera) {
    const auto& ph = photos[idx];
    QString full_path = ph.name;
    QString fname = full_path;
    int slash = full_path.lastIndexOf('/');
    if (slash < 0) slash = full_path.lastIndexOf('\\');
    if (slash >= 0) fname = full_path.mid(slash + 1);

    int n_obs = 0;
    if (idx < static_cast<int>(tracks_->cam_to_tracks_size()))
      n_obs = static_cast<int>(tracks_->cam_track_count(idx));

    show_pick_info(tr("Camera #%1\nName: %2\nObserved 3D points: %3\nFull path:\n%4")
                       .arg(idx).arg(fname).arg(n_obs).arg(full_path));
    tracks_->set_selected_camera(idx);
  } else {
    const auto& tk = track_list[idx];
    QString info = tr("3D Point #%1\nXYZ: (%2, %3, %4)\nObservations: %5\n")
                       .arg(tk.trackId)
                       .arg(tk.x, 0, 'f', 3)
                       .arg(tk.y, 0, 'f', 3)
                       .arg(tk.z, 0, 'f', 3)
                       .arg(tk.obs.size());
    for (size_t k = 0; k < tk.obs.size(); ++k) {
      const auto& ob = tk.obs[k];
      QString cam_name;
      if (ob.photoId >= 0 && ob.photoId < static_cast<int>(photos.size())) {
        QString full = photos[ob.photoId].name;
        int sl = full.lastIndexOf('/');
        if (sl < 0) sl = full.lastIndexOf('\\');
        cam_name = sl >= 0 ? full.mid(sl + 1) : full;
      } else {
        cam_name = tr("cam%1").arg(ob.photoId);
      }
      info += tr("  [%1] cam=%2  uv=(%3, %4)\n")
                  .arg(k).arg(cam_name)
                  .arg(ob.featX, 0, 'f', 1)
                  .arg(ob.featY, 0, 'f', 1);
    }
    show_pick_info(info);
    tracks_->set_selected_track(idx);
  }

  render_widget_->updateGL();
}

} // namespace insight

#include "bundler_viewer_window.moc"
