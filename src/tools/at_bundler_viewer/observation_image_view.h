#pragma once

#include <QResizeEvent>
#include <QWidget>
#include <string>

namespace insight {

namespace render {
class RenderMap2;
class RenderTileImageLayer;
} // namespace render

class FeatureOverlayLayer;

/// Self-contained widget that displays an image with a feature-point overlay.
/// Uses RenderMap2 + RenderTileImageLayer for tiled, zoomable image display
/// and a FeatureOverlayLayer for drawing the feature point crosshair.
class ObservationImageView : public QWidget {
  Q_OBJECT
public:
  explicit ObservationImageView(QWidget* parent = nullptr);
  ~ObservationImageView() override;

  /// Load an image and show the feature point at (u, v) in image-pixel coords.
  /// Returns false if the image file cannot be loaded.
  bool load_image_with_feature(const std::string& image_path,
                               double u, double v,
                               double image_w, double image_h);

  /// Clear the current image and overlay.
  void clear();

protected:
  void resizeEvent(QResizeEvent* event) override;

private:
  render::RenderMap2*           map_           = nullptr;
  render::RenderTileImageLayer* image_layer_   = nullptr;
  FeatureOverlayLayer*          overlay_layer_ = nullptr;
  bool image_loaded_ = false;
};

} // namespace insight
