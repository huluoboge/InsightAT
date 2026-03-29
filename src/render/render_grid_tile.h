/*
@author : Jones
@date: 2017-04-27
@descrpiton:

*/
#pragma once

#include "render_global.h"

#include <QMutex>
#include <QObject>
#include <QPointF>
#include <QRect>
#include <QRectF>
#include <QThread>
#include <map>
#include <memory>
#include <vector>

#include "ImageIO/ImageInfo.h"

namespace insight {

namespace render {

#define COLOR_GL_RGB 0x1907
#define COLOR_GL_RGBA 0x1908

struct TileData {
  // ???????????????????????
  int image_x = 0;
  int image_y = 0;
  int image_width = 0;
  int image_height = 0;

  // ?????????????????????????????????
  /*
  0   1
  3   2
  */
  double x[4]; // x0,x1,x2,x3; left top, right top, right bottom, left bottom
  double y[4];
};

struct TileKey {
  int row_index = 0;
  int column_index = 0;
  int level = 0;
};

struct ImageTile {
  std::vector<TileData> tiles;
  std::vector<void*> buffers;
  int w = 0;
  int h = 0;
  int color_type = COLOR_GL_RGB;

  void destroy_buffers() {
    for (int i = 0; i < int(buffers.size()); ++i) {
      delete[] static_cast<char*>(buffers[i]);
      buffers[i] = nullptr;
    }
  }
};

struct Tile {
  Tile() : pixel_data(nullptr) { pix_type = EnPixelType::PIXEL_Byte; }

  void destroy() {
    dirty = true;
    if (pixel_data) {
      delete[] static_cast<char*>(pixel_data);
    }
    if (rgb) {
      delete[] rgb;
    }
    if (normal) {
      delete[] normal;
    }
    pixel_data = nullptr;
    rgb = nullptr;
    normal = nullptr;
  }

  ~Tile() = default;

  TileKey key;
  TileData tile;
  void* pixel_data = nullptr;
  uint8_t* rgb = nullptr;
  int8_t* normal = nullptr; // ??????
  EnPixelType pix_type = EnPixelType::PIXEL_Byte;
  int band_count = 1;
  bool dirty = false;
  QMutex mutex;
};

/**
 * @brief 某一层级的所有瓦片
 */
struct PyramidLevel {
  std::vector<TileData> tiles;
  int rows = 0;
  int cols = 0;
  int level = 0;
};

typedef std::shared_ptr<ImageTile> ImageTilePtr;

class RENDER_EXPORT RenderGridTile : public QObject {
  Q_OBJECT

public:
  explicit RenderGridTile(QObject* parent = nullptr);

  static uint64_t make_hash(int level, int id);

  static void parse_hash(uint64_t hash, int& level, int& id);

  static std::vector<TileData> create_tiles_by_level(int width, int height, int level, int& columns,
                                                     int& rows, int base_tile_size = 512,
                                                     int base_tile_buffer_pix = 0,
                                                     double* transform = nullptr);
  static void create_tiles_by_level(int width, int height, int level, int& columns, int& rows,
                                    std::vector<TileData>& tiles, int base_tile_size = 512,
                                    int base_tile_buffer_pix = 0, double* transform = nullptr);

  void set_transform(double transform[6]);

  void get_transform(double transform[6]);

  void build_pyramid(int width, int height, int deeps);

  void build_pyramid_auto_deeps(int width, int height, int min_width = 1024);

  void get_whd(int& w, int& h, int& deep) {
    w = width_;
    h = height_;
    deep = deep_;
  }

  bool query_tiles(const QRectF& rect, int level, std::map<uint64_t, TileData>& tiles);

  bool query_tiles(const QRectF& rect, int level, std::vector<Tile*>& tiles);

  int tile_count_in_buffer_pool() const { return pool_tile_count_; }

  void clear_some_tiles(int current_level);

  void clear_all_tiles();

  int base_tile_size() const { return base_tile_size_; }
  void set_base_tile_size(int val) { base_tile_size_ = val; }

  int base_tile_buffer_pix() const { return base_tile_buffer_pix_; }
  void set_base_tile_buffer_pix(int val) { base_tile_buffer_pix_ = val; }

  int max_buffer_pool_tile_size() const { return max_buffer_pool_tile_size_; }
  void set_max_buffer_pool_tile_size(int val) { max_buffer_pool_tile_size_ = val; }

signals:
  void delete_tiles(std::vector<Tile*>);

public slots:
  void on_delete_slots(const std::vector<Tile*>& tiles);

private:
  Tile* get_tile_from_buffer_pool(int row_index, int column_index, int level);

  int get_pool_size();

  int remove_some_tiles_in_one_level(std::vector<Tile*>& vec_tiles);

  std::vector<PyramidLevel> pyramid_;
  int base_tile_size_ = 512;
  int base_tile_buffer_pix_ = 0;

  typedef std::map<int, std::vector<Tile*>> MapLevelTiles;
  MapLevelTiles map_level_tiles_;

  int pool_tile_count_ = 0;
  int width_ = -1;
  int height_ = -1;
  int deep_ = -1;
  double transform_[6]{};
  int max_buffer_pool_tile_size_ = 1000;
};

struct PyramidData {
  std::vector<Tile*> tiles;
};

class RENDER_EXPORT PyramidDataQueue {
public:
  void add_data(PyramidData* data);

  void clear_all_data();

  PyramidData* get_one_data();

private:
  std::vector<PyramidData*> vec_data_;
};

class RENDER_EXPORT TileImageLoader : public QThread {
  Q_OBJECT
signals:
  void send_update_tiles(const PyramidData* p_data);

public:
  static constexpr int k_invalid_value = -9999;

  enum EDataType {
    DATA_RGB = 0,
    DATA_DEM = 1,
  };

  explicit TileImageLoader(QObject* parent);
  ~TileImageLoader() override;

  void set_exit_requested(bool e) { exit_requested_ = e; }

  void run() override;

  bool is_done() const { return done_; }

  void set_image_stream(ImageStream* stream) { image_stream_ = stream; }

  void set_pyramid_tile(RenderGridTile* pyramid_tile) { pyramid_tile_ = pyramid_tile; }

  int type() const { return type_; }
  void set_type(int val) { type_ = val; }

  void do_tasks(std::vector<Tile*>& vec_tiles);

private:
  PyramidDataQueue* data_queue_ = nullptr;

  ImageStream* image_stream_ = nullptr;

  RenderGridTile* pyramid_tile_ = nullptr;

  bool done_ = false;

  int type_ = DATA_RGB;

  bool exit_requested_ = false;
};
} // namespace render

} // namespace insight
