/**
 * @file  bundler_viewer_window.h
 * @brief 独立 Bundler 场景查看窗口（仅依赖 render 库）。
 */
#pragma once

#include <QMainWindow>

namespace insight {
namespace render {
class RenderTracks;
class RenderWidget;
} // namespace render
} // namespace insight

namespace insight {

class BundlerViewerWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit BundlerViewerWindow(QWidget* parent = nullptr);
  ~BundlerViewerWindow() override;

private slots:
  void open_bundler_directory();

  void on_reset_view();
  void on_pivot_toggled(bool visible);
  void on_cameras_toggled(bool visible);
  void on_points_toggled(bool visible);
  void on_frustum_smaller();
  void on_frustum_larger();
  void on_point_size_smaller();
  void on_point_size_larger();

private:
  render::RenderWidget* render_widget_ = nullptr;
  render::RenderTracks* tracks_ = nullptr;
};

} // namespace insight
