#include "render_layer.h"
#include "ImageIO/gdal_utils.h"
#include "render_camera.h"
#include <QString>
#include <QFileInfo>

namespace insight{

namespace render
{
	RenderTileImageLayer::RenderTileImageLayer(QObject *parent)
		: RenderLayer(parent),
		  m_valid(false), m_textureCount(0), m_textureNames(NULL)
	{
		m_tileImageLoader = new TileImageLoader(this);
		m_imageStream = nullptr;
		m_pyramid = new RenderGridTile(this);
		QObject::connect(m_tileImageLoader, SIGNAL(sendUpdateTiles(const PyramidData *)),
						 this, SLOT(receiveUpdateTiles(const PyramidData *)), Qt::DirectConnection);
		m_geoCoordinate = true;
		m_pyramid->setMaxBufferPoolTileSize(m_poolMaxSize);
		m_drawFrame = false;
		m_drawName = false;
	}

	RenderTileImageLayer::~RenderTileImageLayer()
	{
		if (m_tileImageLoader->isRunning())
		{
			m_tileImageLoader->setExist(true);
			m_tileImageLoader->wait();
		}
	}

	void RenderTileImageLayer::toImageCoord(double x, double y, double &ix, double &iy)
	{
		if (m_pyramid)
		{
			double trans[6];
			m_pyramid->getTransform(trans);
			GdalUtils::Geo2Raster(trans, &x, &y);
			ix = x;
			iy = y;
		}
	}

	void RenderTileImageLayer::toPaintCoord(double ix, double iy, double &x, double &y)
	{
		if (m_pyramid)
		{
			double trans[6];
			m_pyramid->getTransform(trans);
			GdalUtils::Raster2Geo(trans, &ix, &iy);
			x = ix;
			y = iy;
		}
	}

	bool RenderTileImageLayer::load(const std::string &file)
	{
		printf("RenderTileImageLayer::load\n");
		if (m_tileImageLoader->isRunning())
		{
			m_tileImageLoader->setExist(true);
			m_tileImageLoader->wait();
		}
		if (m_imageStream)
		{
			m_imageStream->Close();
			delete m_imageStream;
		}
		m_imageStream = new ImageStream;
		bool ok = m_imageStream->Open(file);
		if (!ok)
		{
			return false;
		}
		ImageInfo info = m_imageStream->ImageInformation();
		double trans[6];
		if (info.IsGeoTransformValid())
		{
			info.GetGeoTransForm(trans);
		}
		else
		{
			if (m_geoCoordinate)
			{
				GdalUtils::Init6GeoTransform(trans, info.Rows());
			}
			else
			{
				GdalUtils::Init6Transform(trans);
			}
		}
		//make extent
		int width = info.Columns();
		int height = info.Rows();

		double xs[4] = {0.0, double(width), double(width), 0.0};
		double ys[4] = {0, 0, double(height), double(height)};
		double minx = 0, miny = 0, maxx = 0, maxy = 0;
		for (int i = 0; i < 4; ++i)
		{
			double x = xs[i], y = ys[i];
			double x1 = trans[0] + x * trans[1] + y * trans[2];
			double y1 = trans[3] + x * trans[4] + y * trans[5];
			if (i == 0)
			{
				minx = x1;
				miny = y1;
				maxx = x1;
				maxy = y1;
			}
			else
			{
				minx = std::min(minx, x1);
				miny = std::min(miny, y1);
				maxx = std::max(maxx, x1);
				maxy = std::max(maxy, y1);
			}
		}
		m_extent.setX(minx);
		m_extent.setY(miny);
		m_extent.setWidth(maxx - minx);
		m_extent.setHeight(maxy - miny);
		//m_extent.translate(-trans[0], -trans[3]);
		//����Ϊ�˼��ٴ�����ƫ�ƽ�ƾ���
		setPosition(trans[0], trans[3], 0);
		//m_localOrigin.setX(trans[0]);
		//m_localOrigin.setY(trans[3]);
		trans[0] = 0;
		trans[3] = 0;
		m_pyramid->setTransform(trans);
		m_pyramid->buildPyramidAutoDeeps(info.Columns(), info.Rows(), m_tileSize * 2);
		m_tileImageLoader->setImageStream(m_imageStream);
		m_tileImageLoader->setPyramidTile(m_pyramid);
		m_tileImageLoader->setExist(false);
		m_tileImageLoader->start();
		m_file = QString::fromLocal8Bit(file.c_str());
		QFileInfo fileInfo(m_file);
		m_name = fileInfo.fileName();
		printf("RenderTileImageLayer::load_end\n");

		return true;
	}

	void RenderTileImageLayer::setData(const std::vector<Tile *> &vecTiles)
	{
		m_vecTileTexCoord.clear();
		destroyTextures();

		m_textureCount = int(vecTiles.size());
		m_textureNames = new GLuint[m_textureCount];
		glGenTextures(m_textureCount, m_textureNames);

		for (int i = 0; i < m_textureCount; ++i)
		{
			glBindTexture(GL_TEXTURE_2D, m_textureNames[i]);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			int rgbType = GL_RGB;
			if (vecTiles[i]->m_nBand == 3)
			{
				rgbType = GL_RGB;
			}
			else if (vecTiles[i]->m_nBand == 4)
			{
				rgbType = GL_RGBA;
			}
			if (vecTiles[i]->m_pData)
			{
				glTexImage2D(GL_TEXTURE_2D, 0, rgbType, vecTiles[i]->m_tile.imageWidth, vecTiles[i]->m_tile.imageHeight, 0, rgbType, GL_UNSIGNED_BYTE, vecTiles[i]->m_pData);
			}

			//����tile����������
			m_vecTileTexCoord.push_back(vecTiles[i]->m_tile);
		}
		glBindTexture(GL_TEXTURE_2D, 0);

		m_valid = true;
	}

	void RenderTileImageLayer::draw(RenderContext *rc)
	{
		//Mat4 viewMatrix = rc->camera->viewMatrix();
		/*
		�����޸ģ�
		���� ��Ϊopengl matrix�������⣬��Ҫ������double���������֮���ٸ�opengl
		������Ⱦ�����ź�ƽ��ʱ�����ڴ���������ֶ���
		*/
		//viewMatrix(0, 3) += m_localOrigin.x();
		//viewMatrix(1, 3) += m_localOrigin.y();
		//glMatrixMode(GL_MODELVIEW);
		//glLoadMatrixd(viewMatrix.data());

		//glTranslatef(x, y, 0);
		if (!m_loaderUpdateConnect)
		{
			QObject::connect(m_tileImageLoader, SIGNAL(sendUpdateTiles(const PyramidData *)),
							 rc->widget, SLOT(update()), Qt::QueuedConnection);
			m_loaderUpdateConnect = true;
		}
		m_pyramidDataMutex.lock();
		if (!m_paramidDeque.empty())
		{
			const PyramidData *pData = m_paramidDeque.front();
			m_paramidDeque.pop_front();
			setData(pData->m_pVecTiles);
			delete pData;
			pData = NULL;
		}
		m_pyramidDataMutex.unlock();
		int w = m_imageStream->ImageInformation().Columns();
		int h = m_imageStream->ImageInformation().Rows();
		if (m_valid)
		{
			glEnable(GL_TEXTURE_2D);

#if ENABLE_MASK
			bool enableMask = (m_maskPolygon.size() >= 3);
			if (enableMask)
			{
				glEnable(GL_BLEND);
				glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glColor4d(1, 1, 1, 1);
				glBegin(GL_TRIANGLES);
				for (int i = 0; i < int(m_maskPolygon.size() / 3); ++i)
				{
					glVertex2f(m_maskPolygon[3 * i].x(), m_maskPolygon[3 * i].y());
					glVertex2f(m_maskPolygon[3 * i + 1].x(), m_maskPolygon[3 * i + 1].y());
					glVertex2f(m_maskPolygon[3 * i + 2].x(), m_maskPolygon[3 * i + 2].y());
				}
				glEnd();
				glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_DST_ALPHA);
			}

#endif
			glColor4d(1.0, 1.0, 1.0, 1.0);

			for (int i = 0; i < m_textureCount; ++i)
			{
				glBindTexture(GL_TEXTURE_2D, m_textureNames[i]);

				glColor3f(1.0, 1.0, 1.0);
				glBegin(GL_QUADS);

				glTexCoord2d(0.0, 0.0);
				glVertex3d(m_vecTileTexCoord[i].x[0], m_vecTileTexCoord[i].y[0], 0);
				glTexCoord2d(1.0, 0.0);
				glVertex3d(m_vecTileTexCoord[i].x[1], m_vecTileTexCoord[i].y[1], 0);
				glTexCoord2d(1.0, 1.0);
				glVertex3d(m_vecTileTexCoord[i].x[2], m_vecTileTexCoord[i].y[2], 0);
				glTexCoord2d(0.0, 1.0);
				glVertex3d(m_vecTileTexCoord[i].x[3], m_vecTileTexCoord[i].y[3], 0);

				glEnd();

				bool drawGrid = false;
				if (drawGrid)
				{
					glLineWidth(1);
					glColor3f(1.0, 0.0, 0.0);
					glBegin(GL_LINE_LOOP);

					glVertex3d(m_vecTileTexCoord[i].x[0], m_vecTileTexCoord[i].y[0], 0);
					glVertex3d(m_vecTileTexCoord[i].x[1], m_vecTileTexCoord[i].y[1], 0);
					glVertex3d(m_vecTileTexCoord[i].x[2], m_vecTileTexCoord[i].y[2], 0);
					glVertex3d(m_vecTileTexCoord[i].x[3], m_vecTileTexCoord[i].y[3], 0);
					glEnd();
				}
			}
			glBindTexture(GL_TEXTURE_2D, 0);
			glDisable(GL_TEXTURE_2D);

			// bool drawFrame = false;
			if (m_drawFrame)
			{
				double hh = h;
				glColor3f(1.0, 1.0, 1.0);
				if (m_geoCoordinate)
				{
					hh = -h;
				}
				glBegin(GL_LINE_LOOP);
				glVertex3d(0, 0, 0);
				glVertex3d(w, 0, 0);
				glVertex3d(w, hh, 0);
				glVertex3d(0, hh, 0);
				glEnd();
			}
#if ENABLE_MASK
			if (enableMask)
			{
				glColor4d(0.5, 0.5, 0.5, 1);
				glBegin(GL_TRIANGLES);
				for (int i = 0; i < int(m_maskPolygon.size() / 3); ++i)
				{
					glVertex2f(m_maskPolygon[3 * i].x(), m_maskPolygon[3 * i].y());
					glVertex2f(m_maskPolygon[3 * i + 1].x(), m_maskPolygon[3 * i + 1].y());
					glVertex2f(m_maskPolygon[3 * i + 2].x(), m_maskPolygon[3 * i + 2].y());
				}
				glEnd();
				glDisable(GL_BLEND);
			}
#endif
			if (m_drawName)
			{
				double hh = h;
				glColor3f(1.0, 1.0, 1.0);
				if (m_geoCoordinate)
				{
					hh = -h;
				}
				glColor3f(0.0, 1.0, 0.0);
				rc->widget->renderText(0.0, hh, 0.0, name());
			}
		}
	}

	void RenderTileImageLayer::receiveUpdateTiles(const PyramidData *pData)
	{
		if (pData->m_pVecTiles.empty())
		{
			return;
		}
		m_pyramidDataMutex.lock();
		m_paramidDeque.push_back(pData);
		m_pyramidDataMutex.unlock();
	}

	void RenderTileImageLayer::repaint(RenderContext *rc, const QRectF &worldExtent)
	{
		int w, h, d;
		m_pyramid->getWHD(w, h, d);
		//to local extent
		//��rect��Χ���㵽��ǰ����ϵ
		//֧����ת
		Vec3 pts[4];
		pts[0] = Vec3(worldExtent.x(), worldExtent.y(), 0.0);
		pts[1] = Vec3(worldExtent.x() + worldExtent.width(), worldExtent.y(), 0.0);
		pts[2] = Vec3(worldExtent.x() + worldExtent.width(), worldExtent.y() + worldExtent.height(), 0.0);
		pts[3] = Vec3(worldExtent.x(), worldExtent.y() + worldExtent.height(), 0.0);
		for (int i = 0; i < 4; ++i)
		{
			pts[i] = worldToLocal(pts[i]);
		}
		double minX = pts[0].x(), minY = pts[0].y();
		double maxX = pts[0].x(), maxY = pts[0].y();
		for (int i = 1; i < 3; ++i)
		{
			minX = std::min(minX, pts[i].x());
			maxX = std::max(maxX, pts[i].x());
			minY = std::min(minY, pts[i].y());
			maxY = std::max(maxY, pts[i].y());
		}
		float width = maxX - minX;
		float height = maxY - minY;
		//QRectF localExtent(worldExtent.x() - thisX, worldExtent.y() - thisY, worldExtent.width(), worldExtent.height());
		QRectF localExtent(minX, minY, width, height);

		std::string wktstr = m_imageStream->ImageInformation().Projection();

		std::vector<Tile *> tiles;
		int vw, vh;
		vw = rc->w;
		vh = rc->h;

		double trans[6];
		m_pyramid->getTransform(trans);
		float scale = fabs(localExtent.width() / (vw * trans[1]));
		int level = log(scale) / log(2);
		printf("level = %d, scale = %f\n ", level, scale);
		if (level < 0)
			level = 0;
		if (level >= d)
		{
			level = d - 1;
		}
		QRectF rect = localExtent;
		bool ok = m_pyramid->queryTiles(rect, level, tiles);
		if (ok && !tiles.empty())
		{
			m_tileImageLoader->doTasks(tiles);
		}
	}

	void RenderTileImageLayer::clearCache()
	{
		if (m_pyramid)
			m_pyramid->clearAllTiles();
	}

	void RenderTileImageLayer::destroyTextures()
	{
		if (m_valid)
		{
			//ɾ������
			glDeleteTextures(m_textureCount, m_textureNames);
			delete[] m_textureNames;
			m_textureNames = NULL;
			m_textureCount = 0;
			m_valid = false;
		}
	}

} // namespace render

}//name space insight
