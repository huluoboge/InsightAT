#include "render_grid_tile.h"

#include "ImageIO/ImageStream.h"
#include "ImageIO/gdal_utils.h"

#include <glog/logging.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <mutex>
namespace insight {

namespace render {
extern bool g_exit_render;
// height,widthӰ�񣬹���������
// ����������ǵ��ŵģ�levelΪ0��ʾԭʼӰ��
std::vector<TileData> RenderGridTile::create_tiles_by_level(int width, int height, int level,
                                                            int& columns, int& rows,
                                                            int baseTileSize /*= 512*/,
                                                            int baseTileBufferPix /* = 0*/,
                                                            double* transform) {
  // int baseTileSize =  base_tile_size_;//512 * 512��С��ͼƬ
  std::vector<TileData> tiles;
  create_tiles_by_level(width, height, level, columns, rows, tiles, baseTileSize, baseTileBufferPix,
                        transform);
  return tiles;
}

void RenderGridTile::create_tiles_by_level(int width, int height, int level, int& columns,
                                           int& rows, std::vector<TileData>& tiles,
                                           int baseTileSize /*= 512*/,
                                           int baseTileBufferPix /* = 0*/, double* transform) {
  double* pTrans = transform;
  double geoTrans[6];
  GdalUtils::Init6GeoTransform(geoTrans, height);
  if (pTrans == nullptr) {
    pTrans = geoTrans;
  }
  int factor = pow(2.0, level);
  int tileSize = baseTileSize * factor;
  int tileBuffSize = baseTileBufferPix * factor;

  int image_width = width / factor;
  int image_height = height / factor;
  std::vector<TileData>& blocks = tiles;
  blocks.clear();
  int maxSize = std::max(width, height);
  // tile�±� ˳ʱ��
  // 0 1
  // 3 2
  if (maxSize <= tileSize) {
    TileData tile;
    tile.image_x = 0, tile.image_y = 0, tile.image_width = image_width,
    tile.image_height = image_height;
    double xs[4] = {0, double(width), double(width), 0};
    double ys[4] = {0, 0, double(height), double(height)};
    for (int i = 0; i < 4; ++i) {
      tile.x[i] = pTrans[0] + pTrans[1] * xs[i] + pTrans[2] * ys[i];
      tile.y[i] = pTrans[3] + pTrans[4] * xs[i] + pTrans[5] * ys[i];
    }
    // 		tile.x[0] = pTrans[0] + pTrans[1] * 0 + pTrans[2] * 0;
    // 		tile.y[0] =pTrans[3] + pTrans[4] * 0 + pTrans[5]*  0;
    // 		tile.x[1] = width;					tile.y[1] = height;
    // 		tile.x[2] = width;					tile.y[2] = 0;
    // 		tile.x[3] = 0;						tile.y[3] = 0;
    columns = rows = 1;
    blocks.push_back(tile);
  } else {
    int lastWidth = width % tileSize;
    int lastHeight = height % tileSize;

    columns = (lastWidth == 0) ? width / tileSize : width / tileSize + 1;
    rows = (lastHeight == 0) ? height / tileSize : height / tileSize + 1;
    blocks.reserve(columns * rows);

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < columns; ++c) {
        // ���Ͻǵ�Ϊԭ�������ϵ
        TileData tile;
        tile.image_x = c * baseTileSize;
        tile.image_y = r * baseTileSize;

        tile.x[0] = c * tileSize;
        tile.x[3] = c * tileSize;

        tile.y[0] = r * tileSize;
        tile.y[1] = r * tileSize;

        if (c == columns - 1 && lastWidth != 0) {
          tile.image_width = lastWidth / factor;
          tile.x[1] = tile.x[0] + lastWidth;
          tile.x[2] = tile.x[0] + lastWidth;
        } else {
          tile.image_width = baseTileSize + baseTileBufferPix;
          tile.x[1] = tile.x[0] + tileSize + tileBuffSize;
          tile.x[2] = tile.x[0] + tileSize + tileBuffSize;
        }

        if (r == rows - 1 && lastHeight != 0) {
          tile.image_height = lastHeight / factor;

          tile.y[2] = tile.y[0] + lastHeight;
          tile.y[3] = tile.y[0] + lastHeight;
        } else {
          tile.image_height = baseTileSize + baseTileBufferPix;
          tile.y[2] = tile.y[0] + tileSize + tileBuffSize;
          tile.y[3] = tile.y[0] + tileSize + tileBuffSize;
        }

        // ���㵽���Ͻ�ԭ��
        // tile.image_y += tile.image_height;
        // tile.image_y = image_height - tile.image_y;
        // ����ʵ������

        for (int i = 0; i < 4; ++i) {
          double x = tile.x[i];
          double y = tile.y[i];
          tile.x[i] = pTrans[0] + pTrans[1] * x + pTrans[2] * y;
          tile.y[i] = pTrans[3] + pTrans[4] * x + pTrans[5] * y;
        }
        blocks.push_back(tile);
      }
    }
  }
}

void RenderGridTile::set_transform(double transform[6]) {
  memcpy(transform_, transform, sizeof(double) * 6);
}

void RenderGridTile::get_transform(double transform[6]) {
  memcpy(transform, transform_, sizeof(double) * 6);
}

uint64_t RenderGridTile::make_hash(int level, int id) {
  uint64_t hash = 0;
  uint64_t lLeve = level;
  hash |= (lLeve << 32);
  hash |= id;
  return hash;
}

void RenderGridTile::parse_hash(uint64_t hash, int& level, int& id) {
  level = hash >> 32;
  id = (int)hash;
}

void RenderGridTile::build_pyramid(int width, int height, int deeps) {
  assert(deeps > 0);
  int level = 0;

  pyramid_.clear();
  pyramid_.reserve(deeps);

  while (level < deeps) {
    PyramidLevel pymdLevel;
    pymdLevel.tiles;
    create_tiles_by_level(width, height, level, pymdLevel.cols, pymdLevel.rows, pymdLevel.tiles,
                          base_tile_size_, base_tile_buffer_pix_, transform_);
    pymdLevel.level = level;
    pyramid_.push_back(pymdLevel);
    ++level;
  }
  width_ = width;
  height_ = height;
  deep_ = deeps;
  pool_tile_count_ = 0;
}

void RenderGridTile::build_pyramid_auto_deeps(int width, int height, int min_width /*= 1024*/) {
  int w = width;
  int deeps = 1;
  w /= 2;
  while (w >= min_width) {
    deeps += 1;
    w /= 2;
  }
  build_pyramid(width, height, deeps);
}

bool RenderGridTile::query_tiles(const QRectF& rect, int level,
                                 std::map<uint64_t, TileData>& tiles) {
  if (level >= pyramid_.size() || level < 0) {
    return false;
  }
  // ĳһlevel�����е�tiles
  PyramidLevel& pyramidLevel = pyramid_[level];

  int factor = pow(2.0, level);
  int tileSize = base_tile_size() * factor;
  int rows = pyramidLevel.rows;
  int cols = pyramidLevel.cols;
  int x0 = int(rect.x() - tileSize - 1);
  int y0 = int(rect.y() - tileSize - 1);

  int x1 = int(rect.x() + rect.width() + tileSize + 1);
  int y1 = int(rect.y() + rect.height() + tileSize + 1);

  int cbegin = x0 / tileSize > 0 ? x0 / tileSize : 0;
  int rbegin = y0 / tileSize > 0 ? y0 / tileSize : 0;
  int cend = x1 / tileSize == 0 ? x1 / tileSize : x1 / tileSize + 1;
  int rend = y1 / tileSize == 0 ? y1 / tileSize : y1 / tileSize + 1;
  rend = std::min(rows, rend);
  cend = std::min(cols, cend);

  for (int r = rbegin; r < rend; ++r) {
    for (int c = cbegin; c < cend; ++c) {

      int id = r * cols + c;
      uint64_t key = make_hash(level, id);
      tiles.insert(std::make_pair(key, pyramidLevel.tiles[id]));
    }
  }

  return true;
}

bool RenderGridTile::query_tiles(const QRectF& rect, int nLevel, std::vector<Tile*>& tiles) {
  const int level = nLevel;
  if (level >= pyramid_.size() || level < 0) {
    return false;
  }
  int nBuffer = max_buffer_pool_tile_size();
  if (base_tile_size_ < 512) {
    nBuffer *= 512 / base_tile_size_;
  }
  if (pool_tile_count_ >= nBuffer) {
    clear_some_tiles(nLevel);
  }
  // ĳһlevel�����е�tiles
  PyramidLevel& pyramidLevel = pyramid_[level];

  QPointF lb(rect.x(), rect.y());
  QPointF rt(rect.x() + rect.width(), rect.y() + rect.height());
  QRectF rectf(lb, rt);
  int rows = pyramidLevel.rows;
  int cols = pyramidLevel.cols;

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int id = r * cols + c;

      TileData& tile = pyramidLevel.tiles.at(id);
      QPointF bottomLeft(tile.x[3], tile.y[3]);
      QPointF topRight(tile.x[1], tile.y[1]);
      QRectF r1(bottomLeft, topRight);
      if (rectf.intersects(r1)) {
        Tile* pTile = get_tile_from_buffer_pool(r, c, nLevel);
        if (!pTile) {
          pTile = new Tile;
          int id = r * cols + c;
          pTile->tile = tile;
          pTile->key.row_index = r;
          pTile->key.column_index = c;
          pTile->key.level = nLevel;
          // ���뻺���
          map_level_tiles_[nLevel].push_back(pTile);
          pool_tile_count_++;
        }
        tiles.push_back(pTile);
      }
    }
  }

  return true;
}

RenderGridTile::RenderGridTile(QObject* parent)
    : QObject(parent), base_tile_size_(512), base_tile_buffer_pix_(0) // Ĭ�����ص�
{
  GdalUtils::Init6Transform(transform_);
}

void RenderGridTile::on_delete_slots(const std::vector<Tile*>& tiles) {
  for (int i = 0; i < tiles.size(); ++i) {
    if (tiles[i]->dirty) {
      delete tiles[i];
    }
  }
}

Tile* RenderGridTile::get_tile_from_buffer_pool(int nRowIndex, int nColumnIndex, int nLevel) {
  std::vector<Tile*>& vecLevelTiles = map_level_tiles_[nLevel];

  int nSize = vecLevelTiles.size();
  for (int i = 0; i < nSize; i++) {
    Tile* pTile = vecLevelTiles.at(i);
    if (!pTile) {
      continue;
    }

    if (nRowIndex == pTile->key.row_index && nColumnIndex == pTile->key.column_index) {
      return pTile;
    }
  }

  return NULL;
}

// �õ�������п�ĸ���
int RenderGridTile::get_pool_size() {
  int nPoolSize = 0;
  MapLevelTiles::iterator it;
  for (it = map_level_tiles_.begin(); it != map_level_tiles_.end(); ++it) {
    nPoolSize += it->second.size();
  }
  return nPoolSize;
}

// �Ƚϼ򵥵��ڴ��������
// �����һЩ�������ڴ��еĿ飬��ֹռ���ڴ�̫��
void RenderGridTile::clear_some_tiles(int nCurrentLevel) {
  int nBuffer = max_buffer_pool_tile_size();
  if (base_tile_size_ < 512) {
    nBuffer *= 512 / base_tile_size_;
  }

  int nAllRemoveTilesCount = nBuffer / 2;
  int nBeRemovedCount = 0;

  MapLevelTiles::iterator findIt = map_level_tiles_.find(nCurrentLevel - 2);
  if (findIt != map_level_tiles_.end()) {
    nBeRemovedCount += remove_some_tiles_in_one_level(findIt->second);
  } else {
    findIt = map_level_tiles_.find(nCurrentLevel + 3);
    if (findIt != map_level_tiles_.end()) {
      nBeRemovedCount += remove_some_tiles_in_one_level(findIt->second);
    }
  }

  findIt = map_level_tiles_.find(nCurrentLevel + 2);
  if (findIt != map_level_tiles_.end()) {
    nBeRemovedCount += remove_some_tiles_in_one_level(findIt->second);
  } else {
    findIt = map_level_tiles_.find(nCurrentLevel - 3);
    if (findIt != map_level_tiles_.end()) {
      nBeRemovedCount += remove_some_tiles_in_one_level(findIt->second);
    }
  }
}

void RenderGridTile::clear_all_tiles() {

  for (auto itr = map_level_tiles_.begin(); itr != map_level_tiles_.end(); ++itr) {
    auto& vecTiles = itr->second;
    for (int i = 0; i < vecTiles.size(); ++i) {
      Tile* pTile = vecTiles[i];
      pTile->mutex.lock();
      pTile->destroy(); // ����ɾ���Ļ�������֮ǰ�Ѿ���tile�����߳��У�maybe crack
      pTile->mutex.unlock();
    }
  }

  pool_tile_count_ = 0;
}

int RenderGridTile::remove_some_tiles_in_one_level(std::vector<Tile*>& vecTiles) {
  int nSize = vecTiles.size();
  int nRemoveCout = nSize / 2;
  int nNum = 0;
  std::vector<Tile*>::iterator it;
  for (it = vecTiles.begin(); it != vecTiles.end();) {
    if (nNum <= nRemoveCout) {
      // ����ɾ���Ļ�������֮ǰ�Ѿ���tile�����߳��У�maybe crack
      Tile* pTile = *it;
      if (pTile->mutex.tryLock(1)) {
        // delete pTile;
        pTile->destroy();
        pTile->mutex.unlock();
        // pTile = NULL;
        it = vecTiles.erase(it);
        nNum++;
        pool_tile_count_--;
      } else {
        ++it;
      }
    } else {
      break;
    }
  }

  return nNum;
}

//////////////////////////////////////////////////////////////////////////
static QMutex s_mutex;
void PyramidDataQueue::add_data(PyramidData* data) {
  s_mutex.lock();
  vec_data_.push_back(data);
  s_mutex.unlock();
}

void PyramidDataQueue::clear_all_data() {
  s_mutex.lock();
  vec_data_.clear();
  s_mutex.unlock();
}

PyramidData* PyramidDataQueue::get_one_data() {
  QMutexLocker locker(&s_mutex);
  if (vec_data_.empty()) {
    return nullptr;
  }
  PyramidData* data = vec_data_.back();
  vec_data_.pop_back();
  return data;
}

//////////////////////////////////////////////////////////////////////////
// ��̬�����߳�
std::mutex g_loader;

TileImageLoader::TileImageLoader(QObject* parent) : QThread(parent) {
  data_queue_ = new PyramidDataQueue;
}

TileImageLoader::~TileImageLoader() { delete data_queue_; }

void TileImageLoader::do_tasks(std::vector<Tile*>& vecTiles) {
  data_queue_->clear_all_data();
  int nSize = vecTiles.size();
  int nTileTaskNum = 0;
  PyramidData* data = new PyramidData;
  for (int i = 0; i < nSize; i++) {
    Tile* pTile = vecTiles.at(i);
    data->tiles.push_back(pTile);
  }
  data_queue_->add_data(data);
}

void TileImageLoader::run() {
  while (!done_ && !exit_requested_ && !g_exit_render) {
    if (!data_queue_ || !image_stream_) {
      QThread::msleep(500);
      continue;
    }

    PyramidData* data = data_queue_->get_one_data();

    while (!data && !exit_requested_ && !g_exit_render) {
      g_loader.lock();
      data = data_queue_->get_one_data();
      g_loader.unlock();
      QThread::msleep(500);
    }
    if (exit_requested_)
      return;
    if (!data)
      return;
    if (g_exit_render)
      return;
    g_loader.lock(); // ֻ����һ���߳�io����
    int band[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    bool bOK = false;
    bool bCreateTile = false;
    int nLevels = image_stream_->ImageInformation().Levels();
    int totalW = image_stream_->ImageInformation().Columns();
    int totalH = image_stream_->ImageInformation().Rows();
    int nBandCount = image_stream_->ImageInformation().Bands();
    float demInvalid = TileImageLoader::k_invalid_value;
    if (image_stream_->ImageInformation().HasNoDataValue()) {
      demInvalid = image_stream_->ImageInformation().NoDataValue();
    }
    double trans[6];
    memset(trans, 0, 6 * sizeof(double));
    trans[1] = 1.0;
    trans[5] = -1.0;
    image_stream_->ImageInformation().GetGeoTransForm(trans);
    float ddx = fabs(trans[1]);
    float ddy = fabs(trans[5]);

    if (nBandCount > 3) {
      nBandCount = 3; // 最多使用 3 个波段
    }
    EnPixelType pixType = (EnPixelType)(int)image_stream_->ImageInformation().PixelType();
    int nByte = 1;
    switch (pixType) {
    case PIXEL_Byte:
      nByte = 1;
      break;
    case PIXEL_UInt16:
    case PIXEL_Int16:
      nByte = 2;
      break;
    case PIXEL_UInt32:
    case PIXEL_Int32:
    case PIXEL_Float32:
      nByte = 4;
      break;
    case PIXEL_Float64:
      nByte = 8;
      break;
    default:
      nByte = 0;
    }

    if (nByte == 0) {
      throw std::runtime_error("Not support pix type at" __FILE__);
    }
    for (int i = 0; data && i < data->tiles.size(); ++i) {
      if (1) {
        data->tiles[i]->mutex.lock();
        if (data->tiles[i]->dirty) {
          data->tiles[i]->mutex.unlock();
          continue;
        }
        if (!data->tiles[i]->pixel_data) {
          // printf("Reading datas...");
          const int dataSize = data->tiles[i]->tile.image_width * data->tiles[i]->tile.image_height;
          int scale = pow(2, data->tiles[i]->key.level);
          data->tiles[i]->pixel_data = new uint8_t[dataSize * nBandCount * nByte];
          data->tiles[i]->pix_type = (EnPixelType)(int)(pixType);
          data->tiles[i]->band_count = nBandCount;
          if (data->tiles[i]->key.level >= nLevels) {
            // forece dowload read
            bOK = image_stream_->ReadRange(
                0, data->tiles[i]->tile.image_x * scale, data->tiles[i]->tile.image_y * scale,
                data->tiles[i]->tile.image_width * scale, data->tiles[i]->tile.image_height * scale,
                data->tiles[i]->pixel_data, data->tiles[i]->tile.image_width,
                data->tiles[i]->tile.image_height, data->tiles[i]->tile.image_width,
                data->tiles[i]->tile.image_height, band, nBandCount);
          } else {
            bOK = image_stream_->ReadRange(
                data->tiles[i]->key.level, data->tiles[i]->tile.image_x,
                data->tiles[i]->tile.image_y, data->tiles[i]->tile.image_width,
                data->tiles[i]->tile.image_height, data->tiles[i]->pixel_data,
                data->tiles[i]->tile.image_width, data->tiles[i]->tile.image_height,
                data->tiles[i]->tile.image_width, data->tiles[i]->tile.image_height, band,
                nBandCount);
          }

          // printf("reading scale = %d\n", scale);
          if (!bOK) {
            data->tiles[i]->mutex.unlock();
            LOG(ERROR) << "Read image block failed";
            break;
          } else {
            bCreateTile = true;
          }
          // 扩展为 RGBA
          const bool bForceRGBA = true;
          if (bForceRGBA && nBandCount == 3 && nByte == 1) {
            uint8_t* pByte = new uint8_t[dataSize * 4 * nByte];
            memset(pByte, 255, dataSize * 4 * nByte);
            uint8_t* psrc = (uint8_t*)data->tiles[i]->pixel_data;
            for (int j = 0; j < dataSize; ++j) {
              uint8_t* pRgba = &pByte[4 * j * nByte];
              pRgba[0] = psrc[3 * j + 0];
              pRgba[1] = psrc[3 * j + 1];
              pRgba[2] = psrc[3 * j + 2];
            }
            delete[] psrc;
            data->tiles[i]->pixel_data;
            data->tiles[i]->pixel_data = pByte;
            data->tiles[i]->band_count = 4;
          } else if (bForceRGBA && nBandCount == 1 && nByte == 1) {
            uint8_t* pByte = new uint8_t[dataSize * 4 * nByte];
            memset(pByte, 255, dataSize * 4 * nByte);
            uint8_t* psrc = (uint8_t*)data->tiles[i]->pixel_data;
            for (int j = 0; j < dataSize; ++j) {
              uint8_t* pRgba = &pByte[4 * j * nByte];
              pRgba[0] = psrc[j];
              pRgba[1] = psrc[j];
              pRgba[2] = psrc[j];
            }
            delete[] psrc;
            data->tiles[i]->pixel_data;
            data->tiles[i]->pixel_data = pByte;
            data->tiles[i]->band_count = 4;
          } else if (bForceRGBA && nBandCount == 1 && nByte == 2) {
            uint8_t* pByte = new uint8_t[dataSize * 4];
            memset(pByte, 255, dataSize * 4);
            uint16_t* psrc = (uint16_t*)data->tiles[i]->pixel_data;
            const float MAXUINT = 255.f;
            for (int j = 0; j < dataSize; ++j) {
              uint8_t* pRgba = &pByte[4 * j];
              float a = float(psrc[j]) / MAXUINT * 255;
              a = std::clamp(a, 0.f, 255.f);
              pRgba[0] = a;
              pRgba[1] = a;
              pRgba[2] = a;
            }
            delete[] psrc;
            data->tiles[i]->pixel_data;
            data->tiles[i]->pixel_data = pByte;
            data->tiles[i]->pix_type = PIXEL_Byte;
            data->tiles[i]->band_count = 4;
          } else if (bForceRGBA && nBandCount == 1 && nByte == 4) {
            uint8_t* pByte = new uint8_t[dataSize * 4];
            memset(pByte, 255, dataSize * 4);
            uint32_t* psrc = (uint32_t*)data->tiles[i]->pixel_data;
            const float MAXUINT = 65535.f;
            for (int j = 0; j < dataSize; ++j) {
              uint8_t* pRgba = &pByte[4 * j];
              float a = float(psrc[j]) / MAXUINT * 255;
              a = std::clamp(a, 0.f, 255.f);
              pRgba[0] = a;
              pRgba[1] = a;
              pRgba[2] = a;
            }
            delete[] psrc;
            data->tiles[i]->pixel_data;
            data->tiles[i]->pixel_data = pByte;
            data->tiles[i]->pix_type = PIXEL_Byte;
            data->tiles[i]->band_count = 4;
          }
        }
        data->tiles[i]->mutex.unlock();
      }
    }
    emit send_update_tiles(data);
    g_loader.unlock();
  }
}
} // namespace render

} // namespace insight
