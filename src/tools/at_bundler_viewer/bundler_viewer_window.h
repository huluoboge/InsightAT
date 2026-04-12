#pragma once

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>

namespace insight {
namespace render {
class RenderTracks;
class RenderWidget;
struct BundlerScene;
} // namespace render
} // namespace insight

namespace insight {

class BundlerViewerWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit BundlerViewerWindow(QWidget* parent = nullptr);
  ~BundlerViewerWindow() override;

  /// Load an iter_NNNN series from the given parent directory (called at startup if path given).
  void open_iter_series_from_path(const QString& parent_dir);

protected:
  void keyPressEvent(QKeyEvent* event) override;

private slots:
  void open_bundler_directory();
  void open_iter_series();

  void on_reset_view();
  void on_pivot_toggled(bool visible);
  void on_cameras_toggled(bool visible);
  void on_points_toggled(bool visible);
  void on_frustum_smaller();
  void on_frustum_larger();
  void on_point_size_smaller();
  void on_point_size_larger();

  void on_prev_iter();
  void on_next_iter();
  void on_iter_slider_changed(int value);

  void on_pick_mode_toggled(bool enabled);
  void on_render_widget_clicked(int px, int py);

private:
  /// Load and display the bundler data at iter_dirs_[idx].
  void load_iter_at(int idx);
  /// Update label + slider + button states based on current_iter_idx_.
  void update_iter_controls();

  /// Return the BundlerScene for iter idx, from cache or disk (blocking).
  /// Sets *from_cache=true if no disk I/O was needed (either cached or prefetch completed).
  std::shared_ptr<render::BundlerScene> get_or_load_scene(int idx, bool* from_cache);

  /// Launch background loads for idx ± 1..kPrefetchRadius into scene_cache_.
  void start_prefetch(int idx);

  /// Drain all in-flight prefetch futures (called in destructor and on series change).
  void cancel_prefetch();

  /// Show pick result in the info panel.
  void show_pick_info(const QString& text);

  render::RenderWidget* render_widget_ = nullptr;
  render::RenderTracks* tracks_ = nullptr;

  // ── Current scene (kept for pick queries) ───────────────────────────────────
  std::shared_ptr<render::BundlerScene> current_scene_;

  // ── Iteration series ────────────────────────────────────────────────────────
  std::vector<std::string> iter_dirs_; ///< Sorted list of iter_NNNN sub-directories.
  int current_iter_idx_ = -1;          ///< Index into iter_dirs_, -1 when not in series mode.
  bool dims_warmed_ = false;           ///< True after first fill_render_tracks_from_bundler call.

  // ── BundlerScene cache (keyed by iter index) ────────────────────────────────
  std::mutex cache_mutex_;
  std::unordered_map<int, std::shared_ptr<render::BundlerScene>> scene_cache_;

  // ── Async prefetch futures ──────────────────────────────────────────────────
  std::mutex prefetch_mutex_;
  std::unordered_map<int, std::future<std::shared_ptr<render::BundlerScene>>> prefetch_futures_;

  // Iteration navigation toolbar widgets (hidden until a series is loaded)
  QWidget*     iter_bar_widget_  = nullptr;
  QPushButton* btn_prev_iter_    = nullptr;
  QPushButton* btn_next_iter_    = nullptr;
  QSlider*     iter_slider_      = nullptr;
  QLabel*      iter_label_       = nullptr;

  // Pick mode
  QPushButton* btn_pick_mode_    = nullptr;
  QTextEdit*   pick_info_panel_  = nullptr;
  bool         pick_mode_active_ = false;
};

} // namespace insight
