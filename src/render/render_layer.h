#pragma once
/*
@author: Jones
@date: 2019-04-28
@description:
*/

#include "render_global.h"
#include "render_context.h"
#include "render_grid_tile.h"
#include "render_map2.h"
#include "render_node.h"

#include "ImageIO/ImageStream.h"

#include <QObject>
#include <deque>
#include <vector>
#include <string>
#include <QMutex>
#include <mutex>

namespace insight{

namespace render
{

	class  RenderLayer : public QObject, public RenderNode
	{
		Q_OBJECT
		Q_DISABLE_COPY(RenderLayer)
	public:
		RenderLayer(QObject *parent) : QObject(parent)
		{
			m_visible = true;
		}

		virtual ~RenderLayer() {}

		QString name() const { return m_name; }
		QString file() const { return m_file; }

		bool isVisible() const { return m_visible; }
		void setVisible(bool val) { m_visible = val; }
		virtual void render(RenderContext *rc)
		{
			if (isVisible())
			{
				RenderNode::render(rc);
			}
		}
	public slots:
		virtual void repaint(RenderContext *rc, const QRectF &worldExtent) {}

		QRectF extent() const
		{
			return m_extent;
		}

	protected:
		virtual void draw(RenderContext *rc) {}
		QRectF m_extent;
		QString m_file;
		QString m_name;
		bool m_visible;
	};

	class  RenderTileImageLayer : public RenderLayer
	{
		Q_OBJECT
	public:
		RenderTileImageLayer(QObject *parent = NULL);
		virtual ~RenderTileImageLayer();
		bool load(const std::string &file);
		void setGeoCoord(bool val) { m_geoCoordinate = val; }
		ImageStream *imageStream() { return m_imageStream; }

		void toImageCoord(double x, double y, double &ix, double &iy);
		void toPaintCoord(double ix, double iy, double &x, double &y);
	public slots:
		void setData(const std::vector<Tile *> &vecTiles);
		void receiveUpdateTiles(const PyramidData *pData);
		virtual void repaint(RenderContext *rc, const QRectF &worldExtent);
		void clearCache();

		void setDrawFrame(bool enable) { m_drawFrame = enable; }
		void setDrawName(bool enable) { m_drawName = enable; }

	protected:
		virtual void draw(RenderContext *rc);
		void destroyTextures();

		bool m_valid; //false;

		GLuint *m_textureNames;
		int m_textureCount;

		std::vector<TileData> m_vecTileTexCoord;

		std::deque<const PyramidData *> m_paramidDeque;
		int m_tileSize = 256;
		int m_poolMaxSize = 100;

		TileImageLoader *m_tileImageLoader;
		RenderGridTile *m_pyramid;
		ImageStream *m_imageStream;

		QPointF m_worldPos;

		std::mutex m_pyramidDataMutex;
		bool m_loaderUpdateConnect = false;

		bool m_geoCoordinate;
		bool m_drawFrame;
		bool m_drawName;
	};
} // namespace render

}//name space insight
