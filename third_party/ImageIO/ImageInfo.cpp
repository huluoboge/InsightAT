#include "gdal.h"
#include "ogr_api.h"
#include "cpl_string.h"
#include "gdal_utils.h"
#include "ImageInfo.h"

namespace insight{

ImageInfo::ImageInfo()
{
	Reset();
}

void ImageInfo::Reset()
{
	m_nColumns = 0;
	m_nRows = 0;
	m_nBands = 0;
	m_nPixelBits = 0;
	m_enumPixelType = PIXEL_Byte;
	m_nBlockXSize = 0;
	m_nBlockYSize = 0;
	m_nLevels = 1;
	m_hImage = 0;
	m_dMinValue = 0.0;
	m_dMaxValue = 0.0;
	m_dNoDataVal = 0.0;//-9999.9;
	m_bHasNoDataVal = false;
	m_overviewColumns.clear();
	m_overviewRows.clear();
	m_mapBandColorInterp.clear();
	GdalUtils::Init6Transform(m_geoTransform);
	GdalUtils::InitColorTable(m_rgbaTable, 4, 256);
	m_geoTransformValid = false;
}

int ImageInfo::TileColumns() const
{
	int nTileCols=0;
	if(m_nBlockXSize != 0)
	{
		nTileCols=(m_nColumns + m_nBlockXSize - 1) / m_nBlockXSize;		
	}

	return nTileCols;
}

int ImageInfo::TileRows() const
{
	int nTileRows = 0;
	if(m_nBlockYSize)
	{
		nTileRows = (m_nRows + m_nBlockYSize - 1) / m_nBlockYSize;	
	}

	return nTileRows;
}

int ImageInfo::ColumnsAt(int nLevel) const
{
	if (nLevel == 0)
	{
		return m_nColumns;
	}
	else//overview
	{
		--nLevel;
		if (nLevel <0 || nLevel > m_overviewColumns.size())
		{
			return -1;
		}
		return m_overviewColumns[nLevel];
	}
	//return -1;	
}

int ImageInfo::RowsAt(int nLevel) const
{
	if (nLevel == 0)
	{
		return m_nRows;
	}
	else//overview
	{
		--nLevel;
		if (nLevel <0 || nLevel > m_overviewRows.size())
		{
			return -1;
		}
		return m_overviewRows[nLevel];
	}
	//return -1;	
}

int ImageInfo::PixelBytes() const
{
	int nBytes= m_nPixelBits * m_nBands / 8;
	if(m_nPixelBits * m_nBands % 8 == 0)
	{
		++nBytes;
	}

	return nBytes;
}

void ImageInfo::GetGeoTransForm(double *geoTransfrom) const
{
	for (int i = 0; i < 6; ++i)
	{
		geoTransfrom[i] = m_geoTransform[i];
	}
}

std::string ImageInfo::Projection() const
{
	return m_strProjection;
}

}//name space insight
