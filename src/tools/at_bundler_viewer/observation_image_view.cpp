#include "observation_image_view.h"

#include "feature_overlay_layer.h"
#include "render/render_map2.h"
#include "render/render_layer.h"

#include <glog/logging.h>

#include <QFileInfo>
#include <QResizeEvent>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>

namespace insight {

ObservationImageView::ObservationImageView(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  map_ = new render::RenderMap2(this);
  layout->addWidget(map_);

  // Feature overlay layer – drawn on top of the image.
  // (Safe to add early: its repaint() is a no-op and draw() bails when no
  // marker is set.)
  overlay_layer_ = new FeatureOverlayLayer(map_);
  map_->add_layer(overlay_layer_);

  // Image layer is created lazily in load_image_with_feature() so that the
  // tile layer never receives a repaint() before m_imageStream is set up.
}

ObservationImageView::~ObservationImageView() {
  // Layers and map are children of this widget; Qt manages their lifetime.
}

bool ObservationImageView::load_image_with_feature(const std::string& image_path,
                                                    double u, double v,
                                                    double image_w, double image_h) {
  // Validate file existence.
  const QString qpath = QString::fromStdString(image_path);
  if (!QFileInfo::exists(qpath)) {
    LOG(WARNING) << "Image file not found: " << image_path;
    clear();
    return false;
  }

  // Tear down any previous image layer (load() fresh each time).
  if (image_layer_) {
    map_->layers().removeOne(image_layer_);
    delete image_layer_;
    image_layer_ = nullptr;
  }

  // Create the tiled image layer lazily – this avoids triggering
  // RenderTileImageLayer::repaint() with a null m_imageStream.
  image_layer_ = new render::RenderTileImageLayer(map_);
  image_layer_->setGeoCoord(false);
  image_layer_->setDrawName(true);
  // Insert image layer *before* the overlay so the overlay draws on top.
  {
    auto& layers = map_->layers();
    int ov_idx = layers.indexOf(overlay_layer_);
    if (ov_idx >= 0)
      layers.insert(ov_idx, image_layer_);
    else
      layers.push_back(image_layer_);
  }

  if (!image_layer_->load(image_path)) {
    LOG(WARNING) << "Failed to load image: " << image_path;
    clear();
    return false;
  }

  // ── Flip Y: image data is Y-down (top-left origin), OpenGL is Y-up ──────
  // After load() the tile layer's local coords = pixel coords.
  // Scale(1,-1,1) + translate(0,img_h) maps pixel y=0 → world y=img_h
  // and pixel y=img_h → world y=0, yielding a right-side-up display.
  const double img_h = image_layer_->extent().height();
  if (img_h > 0) {
    image_layer_->set_position(0, img_h, 0);
    image_layer_->scale(1, -1, 1);
  }

  // Draw the feature point crosshair.  v is Y-down image coords; flip to Y-up.
  overlay_layer_->set_feature_point(u, img_h - v, image_w, image_h);

  image_loaded_ = true;

  // Fit the image to the view.  Defer slightly so the widget has a chance to
  // receive its final size from the layout system if it was just shown.
  QTimer::singleShot(0, this, [this]() {
    if (!image_loaded_ || !image_layer_)
      return;
    const QRectF ext = image_layer_->extent();
    if (map_->width() > 0 && map_->height() > 0 && !ext.isNull()) {
      map_->zoom_to_extent(ext);
    }
  });

  map_->updateGL();
  return true;
}

void ObservationImageView::clear() {
  overlay_layer_->clear_feature_points();
  image_loaded_ = false;

  // Remove and destroy the tile layer so repaint won't crash on a stale
  // image stream.
  if (image_layer_) {
    map_->layers().removeOne(image_layer_);
    delete image_layer_;
    image_layer_ = nullptr;
  }

  map_->repaint();
}

void ObservationImageView::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  map_->resize(event->size());
}

} // namespace insight
