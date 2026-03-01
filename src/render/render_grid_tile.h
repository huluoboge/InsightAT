/*
@author : Jones
@date: 2017-04-27
@descrpiton:

*/
#pragma once

#include "render_global.h"

#include <map>
#include <vector>
#include <memory>
#include <QMutex>
#include <QRect>
#include <QObject>
#include <QPointF>
#include <QRectF>
#include <QThread>

#include "ImageIO/ImageInfo.h"


//namespace insight{
namespace insight {

namespace render
{

	#define  COLOR_GL_RGB  0x1907;
	#define  COLOR_GL_RGBA  0x1908;

	struct TileData
	{
		//Ӱ�����Ͻǣ����ȺͿ��ȣ����أ�
		int imageX, imageY;
		int imageWidth, imageHeight;

		//ʵ�ʻ���Ӱ������� ����������ϵΪ���½�����
		/*
		0   1
		3   2
		*/
		double x[4];//x0,x1,x2,x3; left top, right top, right bottom, left bottom
		double y[4];
	};

	struct TileKey
	{
		int nRowIndex;
		int nColumnIndex;
		int nLevel;
	};

	struct ImageTile
	{
		std::vector<TileData> tiles;
		std::vector<void *> buffers;
		int w = 0;
		int h = 0;
		int colorType = COLOR_GL_RGB;

		void destoryBuffers() {
			for (int i = 0; i < buffers.size(); ++i) {
				delete[] (char*)buffers[i];
				buffers[i] = nullptr;
			}
		}
	};

	struct Tile
	{
		Tile() : m_pData(NULL)
		{
			m_pixType = EnPixelType::PIXEL_Byte;
		}

		void destory() {
			m_bDirty = true;
            if (m_pData) { delete[] (char*)m_pData; }
			if (m_rgb) { delete[] m_rgb; }
			if (m_normal) { delete[] m_normal; }
			m_pData = NULL;
			m_rgb = NULL;
			m_normal = NULL;
		}

		~Tile()
		{

		}
		TileKey m_key;
        TileData m_tile;
		void* m_pData;
		uint8_t *m_rgb = NULL;
		int8_t *m_normal = NULL;//������
		EnPixelType m_pixType;
		int m_nBand = 1;
		bool m_bDirty = false;
		QMutex m_mutex;

	};
	/**
	 * @brief ĳһ�㼶��������Ƭ
	 */
	struct PyramidLevel
	{
		std::vector<TileData> tiles;
		int rows;
		int cols;
		int level;
	};

	typedef std::shared_ptr<ImageTile> ImageTilePtr;

	class  RenderGridTile : public QObject
	{
		Q_OBJECT


	public:
		RenderGridTile(QObject* parent = NULL);


		static uint64_t makeHash(int level, int id);


		static void parseHash(uint64_t hash, int &level, int &id);

		static std::vector<TileData> createTilesByLevel(int width, int height, int level, int &columns, int &rows, int baseTileSize = 512, int baseTileBufferPix = 0, double *transform = nullptr);
		static void createTilesByLevel(int width, int height, int level, int &columns, int &rows, std::vector<TileData> &tiles, int baseTileSize = 512, int baseTileBufferPix = 0, double *transform = nullptr);



		void setTransform(double transform[6]);

		void getTransform(double transform[6]);

		void buildPyramid(int width, int height, int deeps);


		void buildPyramidAutoDeeps(int width, int height, int min_width = 1024);

		void getWHD(int &w, int &h, int &deep) {
			w = m_width;
			h = m_height;
			deep = m_deep;
		}
		bool queryTiles(const QRectF &rect, int level, std::map<uint64_t, TileData> &tiles);

		bool queryTiles(const QRectF &rect, int level, std::vector<Tile*> &tiles);
		//cache

		int getTileCountInBufferPool() { return m_nPoolTileCount; }

		void clearSomeTiles(int nCurrentLevel);

		void clearAllTiles();

		int BaseTileSize() const { return m_baseTileSize; }
		void BaseTileSize(int val) { m_baseTileSize = val; }
		int BaseTileBufferPix() const { return m_baseTileBufferPix; }
		void BaseTileBufferPix(int val) { m_baseTileBufferPix = val; }

		int maxBufferPoolTileSize() const { return g_nMaxBufferPoolTileSize; }
		void setMaxBufferPoolTileSize(int val) { g_nMaxBufferPoolTileSize = val; }
	signals:
		void deleteTiles(std::vector<Tile*>);
	public slots:
		void onDeleteSlots(const std::vector<Tile*> &tiles);
	private:
		Tile* getTileFromBufferPool(int nRowIndex, int nColumnIndex, int nLevel);

		int getPoolSize();

		int removeSomeTilesInOneLevel(std::vector<Tile*>& vecTiles);

	private:
		std::vector<PyramidLevel> m_pyramid;
        int m_baseTileSize;
        int m_baseTileBufferPix;

		//cache
		typedef std::map<int, std::vector<Tile*>> MapLevelTiles;
		MapLevelTiles m_mapLevelTiles;

		int m_nPoolTileCount;
		int m_width = -1;
		int m_height = -1;
		int m_deep = -1;
		double m_transform[6];
		int g_nMaxBufferPoolTileSize = 1000;
	};

	struct PyramidData
	{
		std::vector<Tile*> m_pVecTiles;
	};


	class  PyramidDataQueue
	{
	public:

		void addData(PyramidData* data);

		void clearAllData();

		PyramidData* getOneData();

	private:
		std::vector<PyramidData*> m_vecData;
	};

	class  TileImageLoader : public QThread
	{
		Q_OBJECT
	signals:
		void sendUpdateTiles(const PyramidData* pData);
	public:
		enum {	
			INVLIAD_VALUE = -9999,
		};
		enum EDataType{
            DATA_RGB = 0,
            DATA_DEM = 1,
		};
		TileImageLoader(QObject *parent);
		~TileImageLoader();

		void setExist(bool e) { m_exit = e; }
		virtual void run();

		bool getDone() { return m_done; }

		void setImageStream(ImageStream* pStream) { m_pImageStream = pStream; }

		void setPyramidTile(RenderGridTile * pPyramidTile) { m_pPyramidTile = pPyramidTile; }

		int Type() const { return m_type; }
		void Type(int val) { m_type = val; }

		void doTasks(std::vector<Tile*>& vecTiles);


	private:
		PyramidDataQueue* m_pDataQueue;

		ImageStream* m_pImageStream;

		RenderGridTile* m_pPyramidTile;

		bool m_done = false;

		int m_type = DATA_RGB;

		bool m_exit = false;
	};
}

}
//}//name space insight
