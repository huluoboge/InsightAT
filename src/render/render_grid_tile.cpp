#include "render_grid_tile.h"

#include "ImageIO/ImageStream.h"
#include "ImageIO/gdal_utils.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <mutex>

#include "glog/logging.h"
namespace insight{

namespace render {

template <typename T>
inline T clamp(const T& val, const T& min, const T& max)
{
    return std::max(min, std::min(val, max));
    //(val < min) ? val : ((val>max) ? val : max);
}
extern bool g_exit_render;
std::vector<TileData> RenderGridTile::createTilesByLevel(
    int width, int height, int level, int& columns, int& rows,
    int baseTileSize /*= 512*/, int baseTileBufferPix /* = 0*/,
    double* transform)
{
    // int baseTileSize =  m_baseTileSize;//512 * 512��С��ͼƬ
    std::vector<TileData> tiles;
    createTilesByLevel(width, height, level, columns, rows, tiles, baseTileSize, baseTileBufferPix, transform);
    return tiles;
}

void RenderGridTile::createTilesByLevel(int width, int height, int level, int& columns, int& rows,
    std::vector<TileData>& tiles, int baseTileSize /*= 512*/, int baseTileBufferPix /* = 0*/, double* transform)
{
    double* pTrans = transform;
    double geoTrans[6];
    GdalUtils::Init6GeoTransform(geoTrans, height);
    if (pTrans == nullptr) {
        pTrans = geoTrans;
    }
    int factor = pow(2.0, level);
    int tileSize = baseTileSize * factor;
    int tileBuffSize = baseTileBufferPix * factor;

    int imageWidth = width / factor;
    int imageHeight = height / factor;
    std::vector<TileData>& blocks = tiles;
    blocks.clear();
    int maxSize = std::max(width, height);
    // tile�±� ˳ʱ��
    // 0 1
    // 3 2
    if (maxSize <= tileSize) {
        TileData tile;
        tile.imageX = 0, tile.imageY = 0, tile.imageWidth = imageWidth, tile.imageHeight = imageHeight;
        double xs[4] = { 0, double(width), double(width), 0 };
        double ys[4] = { 0, 0, double(height), double(height) };
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
                tile.imageX = c * baseTileSize;
                tile.imageY = r * baseTileSize;

                tile.x[0] = c * tileSize;
                tile.x[3] = c * tileSize;

                tile.y[0] = r * tileSize;
                tile.y[1] = r * tileSize;

                if (c == columns - 1 && lastWidth != 0) {
                    tile.imageWidth = lastWidth / factor;
                    tile.x[1] = tile.x[0] + lastWidth;
                    tile.x[2] = tile.x[0] + lastWidth;
                } else {
                    tile.imageWidth = baseTileSize + baseTileBufferPix;
                    tile.x[1] = tile.x[0] + tileSize + tileBuffSize;
                    tile.x[2] = tile.x[0] + tileSize + tileBuffSize;
                }

                if (r == rows - 1 && lastHeight != 0) {
                    tile.imageHeight = lastHeight / factor;

                    tile.y[2] = tile.y[0] + lastHeight;
                    tile.y[3] = tile.y[0] + lastHeight;
                } else {
                    tile.imageHeight = baseTileSize + baseTileBufferPix;
                    tile.y[2] = tile.y[0] + tileSize + tileBuffSize;
                    tile.y[3] = tile.y[0] + tileSize + tileBuffSize;
                }

                // ���㵽���Ͻ�ԭ��
                // tile.imageY += tile.imageHeight;
                // tile.imageY = imageHeight - tile.imageY;
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

void RenderGridTile::setTransform(double transform[6])
{
    memcpy(m_transform, transform, sizeof(double) * 6);
}

void RenderGridTile::getTransform(double transform[6])
{
    memcpy(transform, m_transform, sizeof(double) * 6);
}

uint64_t RenderGridTile::makeHash(int level, int id)
{
    uint64_t hash = 0;
    uint64_t lLeve = level;
    hash |= (lLeve << 32);
    hash |= id;
    return hash;
}

void RenderGridTile::parseHash(uint64_t hash, int& level, int& id)
{
    level = hash >> 32;
    id = (int)hash;
}

void RenderGridTile::buildPyramid(int width, int height, int deeps)
{
    assert(deeps > 0);
    int level = 0;

    m_pyramid.clear();
    m_pyramid.reserve(deeps);

    while (level < deeps) {
        PyramidLevel pymdLevel;
        pymdLevel.tiles;
        createTilesByLevel(width, height, level, pymdLevel.cols, pymdLevel.rows, pymdLevel.tiles, m_baseTileSize, m_baseTileBufferPix, m_transform);
        pymdLevel.level = level;
        m_pyramid.push_back(pymdLevel);
        ++level;
    }
    m_width = width;
    m_height = height;
    m_deep = deeps;
    m_nPoolTileCount = 0;
}

void RenderGridTile::buildPyramidAutoDeeps(int width, int height, int min_width /*= 1024*/)
{
    int w = width;
    int deeps = 1;
    w /= 2;
    while (w >= min_width) {
        deeps += 1;
        w /= 2;
    }
    buildPyramid(width, height, deeps);
}

bool RenderGridTile::queryTiles(const QRectF& rect, int level, std::map<uint64_t, TileData>& tiles)
{
    if (level >= m_pyramid.size() || level < 0) {
        return false;
    }
    // ĳһlevel�����е�tiles
    PyramidLevel& pyramidLevel = m_pyramid[level];

    int factor = pow(2.0, level);
    int tileSize = BaseTileSize() * factor;
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
            uint64_t key = makeHash(level, id);
            tiles.insert(std::make_pair(key, pyramidLevel.tiles[id]));
        }
    }

    return true;
}

bool RenderGridTile::queryTiles(const QRectF& rect, int nLevel, std::vector<Tile*>& tiles)
{
    const int level = nLevel;
    if (level >= m_pyramid.size() || level < 0) {
        return false;
    }
    int nBuffer = maxBufferPoolTileSize();
    if (m_baseTileSize < 512) {
        nBuffer *= 512 / m_baseTileSize;
    }
    if (m_nPoolTileCount >= nBuffer) {
        clearSomeTiles(nLevel);
    }
    // ĳһlevel�����е�tiles
    PyramidLevel& pyramidLevel = m_pyramid[level];

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
                Tile* pTile = getTileFromBufferPool(r, c, nLevel);
                if (!pTile) {
                    pTile = new Tile;
                    int id = r * cols + c;
                    pTile->m_tile = tile;
                    pTile->m_key.nRowIndex = r;
                    pTile->m_key.nColumnIndex = c;
                    pTile->m_key.nLevel = nLevel;
                    // ���뻺���
                    m_mapLevelTiles[nLevel].push_back(pTile);
                    m_nPoolTileCount++;
                }
                tiles.push_back(pTile);
            }
        }
    }

    return true;
}

RenderGridTile::RenderGridTile(QObject* parent)
    : QObject(parent)
    , m_baseTileSize(512)
    , m_baseTileBufferPix(0) // Ĭ�����ص�
{
    GdalUtils::Init6Transform(m_transform);
}

void RenderGridTile::onDeleteSlots(const std::vector<Tile*>& tiles)
{
    for (int i = 0; i < tiles.size(); ++i) {
        if (tiles[i]->m_bDirty) {
            delete tiles[i];
        }
    }
}

Tile* RenderGridTile::getTileFromBufferPool(int nRowIndex, int nColumnIndex, int nLevel)
{
    std::vector<Tile*>& vecLevelTiles = m_mapLevelTiles[nLevel];

    int nSize = vecLevelTiles.size();
    for (int i = 0; i < nSize; i++) {
        Tile* pTile = vecLevelTiles.at(i);
        if (!pTile) {
            continue;
        }

        if (nRowIndex == pTile->m_key.nRowIndex && nColumnIndex == pTile->m_key.nColumnIndex) {
            return pTile;
        }
    }

    return NULL;
}

// �õ�������п�ĸ���
int RenderGridTile::getPoolSize()
{
    int nPoolSize = 0;
    MapLevelTiles::iterator it;
    for (it = m_mapLevelTiles.begin(); it != m_mapLevelTiles.end(); ++it) {
        nPoolSize += it->second.size();
    }
    return nPoolSize;
}

// �Ƚϼ򵥵��ڴ��������
// �����һЩ�������ڴ��еĿ飬��ֹռ���ڴ�̫��
void RenderGridTile::clearSomeTiles(int nCurrentLevel)
{
    int nBuffer = maxBufferPoolTileSize();
    if (m_baseTileSize < 512) {
        nBuffer *= 512 / m_baseTileSize;
    }

    int nAllRemoveTilesCount = nBuffer / 2;
    int nBeRemovedCount = 0;

    MapLevelTiles::iterator findIt = m_mapLevelTiles.find(nCurrentLevel - 2);
    if (findIt != m_mapLevelTiles.end()) {
        nBeRemovedCount += removeSomeTilesInOneLevel(findIt->second);
    } else {
        findIt = m_mapLevelTiles.find(nCurrentLevel + 3);
        if (findIt != m_mapLevelTiles.end()) {
            nBeRemovedCount += removeSomeTilesInOneLevel(findIt->second);
        }
    }

    findIt = m_mapLevelTiles.find(nCurrentLevel + 2);
    if (findIt != m_mapLevelTiles.end()) {
        nBeRemovedCount += removeSomeTilesInOneLevel(findIt->second);
    } else {
        findIt = m_mapLevelTiles.find(nCurrentLevel - 3);
        if (findIt != m_mapLevelTiles.end()) {
            nBeRemovedCount += removeSomeTilesInOneLevel(findIt->second);
        }
    }
}

void RenderGridTile::clearAllTiles()
{

    for (auto itr = m_mapLevelTiles.begin(); itr != m_mapLevelTiles.end(); ++itr) {
        auto& vecTiles = itr->second;
        for (int i = 0; i < vecTiles.size(); ++i) {
            Tile* pTile = vecTiles[i];
            pTile->m_mutex.lock();
            pTile->destory(); // ����ɾ���Ļ�������֮ǰ�Ѿ���tile�����߳��У�maybe crack
            pTile->m_mutex.unlock();
        }
    }

    m_nPoolTileCount = 0;
}

int RenderGridTile::removeSomeTilesInOneLevel(std::vector<Tile*>& vecTiles)
{
    int nSize = vecTiles.size();
    int nRemoveCout = nSize / 2;
    int nNum = 0;
    std::vector<Tile*>::iterator it;
    for (it = vecTiles.begin(); it != vecTiles.end();) {
        if (nNum <= nRemoveCout) {
            // ����ɾ���Ļ�������֮ǰ�Ѿ���tile�����߳��У�maybe crack
            Tile* pTile = *it;
            if (pTile->m_mutex.tryLock(1)) {
                // delete pTile;
                pTile->destory();
                pTile->m_mutex.unlock();
                // pTile = NULL;
                it = vecTiles.erase(it);
                nNum++;
                m_nPoolTileCount--;
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
void PyramidDataQueue::addData(PyramidData* data)
{
    s_mutex.lock();
    m_vecData.push_back(data);
    s_mutex.unlock();
}

void PyramidDataQueue::clearAllData()
{
    s_mutex.lock();
    m_vecData.clear();
    s_mutex.unlock();
}

PyramidData* PyramidDataQueue::getOneData()
{
    QMutexLocker locker(&s_mutex);
    if (m_vecData.empty()) {
        return nullptr;
    }
    PyramidData* data = m_vecData.back();
    m_vecData.pop_back();
    return data;
}

//////////////////////////////////////////////////////////////////////////
// ��̬�����߳�
std::mutex g_loader;

TileImageLoader::TileImageLoader(QObject* parent)
    : QThread(parent)
{
    m_pDataQueue = new PyramidDataQueue;
}

TileImageLoader::~TileImageLoader()
{
    delete m_pDataQueue;
}

void TileImageLoader::doTasks(std::vector<Tile*>& vecTiles)
{
    m_pDataQueue->clearAllData();
    int nSize = vecTiles.size();
    int nTileTaskNum = 0;
    PyramidData* data = new PyramidData;
    for (int i = 0; i < nSize; i++) {
        Tile* pTile = vecTiles.at(i);
        data->m_pVecTiles.push_back(pTile);
    }
    m_pDataQueue->addData(data);
}

void TileImageLoader::run()
{
    while (!m_done && !m_exit && !g_exit_render) {
        if (!m_pDataQueue || !m_pImageStream) {
            QThread::msleep(500);
            continue;
        }

        PyramidData* data = m_pDataQueue->getOneData();

        while (!data && !m_exit && !g_exit_render) {
            g_loader.lock();
            data = m_pDataQueue->getOneData();
            g_loader.unlock();
            QThread::msleep(500);
        }
        if (m_exit)
            return;
        if (!data)
            return;
        if (g_exit_render)
            return;
        g_loader.lock(); // ֻ����һ���߳�io����
        int band[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };

        bool bOK = false;
        bool bCreateTile = false;
        int nLevels = m_pImageStream->ImageInformation().Levels();
        int totalW = m_pImageStream->ImageInformation().Columns();
        int totalH = m_pImageStream->ImageInformation().Rows();
        int nBandCount = m_pImageStream->ImageInformation().Bands();
        float demInvalid = INVLIAD_VALUE;
        if (m_pImageStream->ImageInformation().HasNoDataValue()) {
            demInvalid = m_pImageStream->ImageInformation().NoDataValue();
        }
        double trans[6];
        memset(trans, 0, 6 * sizeof(double));
        trans[1] = 1.0;
        trans[5] = -1.0;
        m_pImageStream->ImageInformation().GetGeoTransForm(trans);
        float ddx = fabs(trans[1]);
        float ddy = fabs(trans[5]);

        if (nBandCount > 3) {
            nBandCount = 3; // ����ȡ��������
        }
        EnPixelType pixType = (EnPixelType)(int)m_pImageStream->ImageInformation().PixelType();
        int nByte = 1;
        switch (pixType) {
        case PIXEL_Byte:
            nByte = 1;
            break;
        case PIXEL_UInt16:
            printf("PIXEL_UInt16\n");
        case PIXEL_Int16:
            printf("PIXEL_Int16\n");
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
        printf("1\n");
        bool OK = true;
        for (int i = 0; data && i < data->m_pVecTiles.size(); ++i) {
            if (1) {
                data->m_pVecTiles[i]->m_mutex.lock();
                if (data->m_pVecTiles[i]->m_bDirty) {
                    data->m_pVecTiles[i]->m_mutex.unlock();
                    continue;
                }
                if (!data->m_pVecTiles[i]->m_pData) {
                    // printf("Reading datas...");
                    const int dataSize = data->m_pVecTiles[i]->m_tile.imageWidth * data->m_pVecTiles[i]->m_tile.imageHeight;
                    int scale = pow(2, data->m_pVecTiles[i]->m_key.nLevel);
                    data->m_pVecTiles[i]->m_pData = new uint8_t[dataSize * nBandCount * nByte];
                    data->m_pVecTiles[i]->m_pixType = (EnPixelType)(int)(pixType);
                    data->m_pVecTiles[i]->m_nBand = nBandCount;
                    if (data->m_pVecTiles[i]->m_key.nLevel >= nLevels) {
                        // forece dowload read
                        bOK = m_pImageStream->ReadRange(0,
                            data->m_pVecTiles[i]->m_tile.imageX * scale,
                            data->m_pVecTiles[i]->m_tile.imageY * scale,
                            data->m_pVecTiles[i]->m_tile.imageWidth * scale,
                            data->m_pVecTiles[i]->m_tile.imageHeight * scale,
                            data->m_pVecTiles[i]->m_pData,
                            data->m_pVecTiles[i]->m_tile.imageWidth,
                            data->m_pVecTiles[i]->m_tile.imageHeight,
                            data->m_pVecTiles[i]->m_tile.imageWidth,
                            data->m_pVecTiles[i]->m_tile.imageHeight,
                            band, nBandCount);
                    } else {
                        bOK = m_pImageStream->ReadRange(data->m_pVecTiles[i]->m_key.nLevel,
                            data->m_pVecTiles[i]->m_tile.imageX,
                            data->m_pVecTiles[i]->m_tile.imageY,
                            data->m_pVecTiles[i]->m_tile.imageWidth,
                            data->m_pVecTiles[i]->m_tile.imageHeight,
                            data->m_pVecTiles[i]->m_pData,
                            data->m_pVecTiles[i]->m_tile.imageWidth,
                            data->m_pVecTiles[i]->m_tile.imageHeight,
                            data->m_pVecTiles[i]->m_tile.imageWidth,
                            data->m_pVecTiles[i]->m_tile.imageHeight, band, nBandCount);
                    }

                    // printf("reading scale = %d\n", scale);
                    if (!bOK) {
                        data->m_pVecTiles[i]->m_mutex.unlock();
                        LOG(ERROR) << "Read image block failed ��";
                        break;
                    } else {
                        bCreateTile = true;
                    }
                    // rgb ��ǿ��ʹ��rgba
                    const bool bForceRGBA = true;
                    if (bForceRGBA && nBandCount == 3 && nByte == 1) {
                        printf("2\n");

                        uint8_t* pByte = new uint8_t[dataSize * 4 * nByte];
                        memset(pByte, 255, dataSize * 4 * nByte);
                        uint8_t* psrc = (uint8_t*)data->m_pVecTiles[i]->m_pData;
                        for (int j = 0; j < dataSize; ++j) {
                            uint8_t* pRgba = &pByte[4 * j * nByte];
                            pRgba[0] = psrc[3 * j + 0];
                            pRgba[1] = psrc[3 * j + 1];
                            pRgba[2] = psrc[3 * j + 2];
                        }
                        delete[] psrc;
                        data->m_pVecTiles[i]->m_pData;
                        data->m_pVecTiles[i]->m_pData = pByte;
                        data->m_pVecTiles[i]->m_nBand = 4;
                    } else if (bForceRGBA && nBandCount == 1 && nByte == 1) {
                        printf("3\n");

                        uint8_t* pByte = new uint8_t[dataSize * 4 * nByte];
                        memset(pByte, 255, dataSize * 4 * nByte);
                        uint8_t* psrc = (uint8_t*)data->m_pVecTiles[i]->m_pData;
                        for (int j = 0; j < dataSize; ++j) {
                            uint8_t* pRgba = &pByte[4 * j * nByte];
                            pRgba[0] = psrc[j];
                            pRgba[1] = psrc[j];
                            pRgba[2] = psrc[j];
                        }
                        delete[] psrc;
                        data->m_pVecTiles[i]->m_pData;
                        data->m_pVecTiles[i]->m_pData = pByte;
                        data->m_pVecTiles[i]->m_nBand = 4;
                    } else if (bForceRGBA && nBandCount == 1 && nByte == 2) {
                        printf("4\n");
                        uint8_t* pByte = new uint8_t[dataSize * 4];
                        memset(pByte, 255, dataSize * 4);
                        uint16_t* psrc = (uint16_t*)data->m_pVecTiles[i]->m_pData;
                        const float MAXUINT = 255.f;
                        for (int j = 0; j < dataSize; ++j) {
                            uint8_t* pRgba = &pByte[4 * j];
                            float a = float(psrc[j]) / MAXUINT * 255;
                            a = clamp<float>(a, 0, 255);
                            pRgba[0] = a;
                            pRgba[1] = a;
                            pRgba[2] = a;
                        }
                        delete[] psrc;
                        data->m_pVecTiles[i]->m_pData;
                        data->m_pVecTiles[i]->m_pData = pByte;
                        data->m_pVecTiles[i]->m_pixType = PIXEL_Byte;
                        data->m_pVecTiles[i]->m_nBand = 4;
                        printf("%d,%d\n", data->m_pVecTiles[i]->m_tile.imageWidth, data->m_pVecTiles[i]->m_tile.imageHeight);
                    } else if (bForceRGBA && nBandCount == 1 && nByte == 4) {
                        printf("4\n");
                        uint8_t* pByte = new uint8_t[dataSize * 4];
                        memset(pByte, 255, dataSize * 4);
                        uint32_t* psrc = (uint32_t*)data->m_pVecTiles[i]->m_pData;
                        const float MAXUINT = 65535.f;
                        for (int j = 0; j < dataSize; ++j) {
                            uint8_t* pRgba = &pByte[4 * j];
                            float a = float(psrc[j]) / MAXUINT * 255;
                            a = clamp<float>(a, 0, 255);
                            pRgba[0] = a;
                            pRgba[1] = a;
                            pRgba[2] = a;
                        }
                        delete[] psrc;
                        data->m_pVecTiles[i]->m_pData;
                        data->m_pVecTiles[i]->m_pData = pByte;
                        data->m_pVecTiles[i]->m_pixType = PIXEL_Byte;
                        data->m_pVecTiles[i]->m_nBand = 4;
                        printf("%d,%d\n", data->m_pVecTiles[i]->m_tile.imageWidth, data->m_pVecTiles[i]->m_tile.imageHeight);
                    }
                }
                data->m_pVecTiles[i]->m_mutex.unlock();
            }
        }
        emit sendUpdateTiles(data);
        g_loader.unlock();
    }
}
} // namespace render

}//name space insight
