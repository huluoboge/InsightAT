#pragma once

#include "render/render_layer.h"

namespace insight {

/// Draws a feature-point crosshair marker on top of a tiled image layer.
/// Used with RenderMap2 + RenderTileImageLayer to show where a 3D point
/// projects onto an image.
class FeatureOverlayLayer : public render::RenderLayer {
  Q_OBJECT
public:
  explicit FeatureOverlayLayer(QObject* parent = nullptr);

  /// Set the feature point to draw at (u, v) in image pixel coordinates.
  void set_feature_point(double u, double v, double image_w, double image_h);

  /// Remove all markers.
  void clear_feature_points();

protected:
  void draw(render::RenderContext* rc) override;

private:
  struct Marker {
    double u = 0.0;
    double v = 0.0;
    bool valid = false;
  };
  Marker marker_;
  double img_w_ = 0.0;
  double img_h_ = 0.0;
};

} // namespace insight
