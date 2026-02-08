#include "gdal.h"
#include "ogr_api.h"
#include "cpl_string.h"
#include "ImageInfo.h"
#include "ImageStream.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <sstream>
#include <thread> // std::thread
#include <mutex>  // std::mutex

#include "gdal_utils.h"

#include <glog/logging.h>
#include <glog/log_severity.h>
// #include "image_converter.hpp"
#include <cpl_conv.h>

namespace insight{

static std::mutex g_gdal_mutex; // locks access to counter

#ifdef min
#undef min
#undef max
#endif
bool g_isGDALInitialized(false);

static std::mutex g_gdal_error_mutex;

std::string g_gdal_error_str;

void GdalUtils::DestoryGDAL()
{
	GDALDestroyDriverManager();
}

void GdalUtils::SetDataPath(const std::string &path)
{
	CPLSetConfigOption("GDAL_DATA", path.c_str());
}

bool wIsGDALInitialized()
{
	return g_isGDALInitialized;
}

int wGetGDALVersion()
{
	return atoi(GDALVersionInfo("VERSION_NUM"));
}

void wSetGDALConfigOption(const char *pszKey, const char *pszValue)
{
	CPLSetConfigOption(pszKey, pszValue);
}

void wGeo2Raster(const double *pTransform, double *x, double *y)
{
	double X = *x;
	double Y = *y;
	const double *T = pTransform;

	double imX = (T[5] * (X - T[0]) - T[2] * (Y - T[3])) / (T[5] * T[1] - T[4] * T[2]);
	double imY = (T[4] * (X - T[0]) - T[1] * (Y - T[3])) / (T[4] * T[2] - T[5] * T[1]);
	*x = imX;
	*y = imY;
	// 	a=X-T[0];b=Y-T[3];
	// 	*y=(a*T[4]-b*T[1])/(T[2]*T[4]-T[1]*T[5]);
	// 	*x=(a-T[2]*Y)/T[1];
}

void wRaster2Geo(const double *pTransform, double *x, double *y)
{
	double tx = *x, ty = *y;
	*x = pTransform[0] + pTransform[1] * tx + pTransform[2] * ty;
	*y = pTransform[3] + pTransform[4] * tx + pTransform[5] * ty;
}

void wStatisticRaster(const char *lpstrFilePath, double *MinVal, double *MaxVal, double *MeanVal /*=NULL*/, double *StdDev /*=NULL*/)
{
	GdalUtils::InitGDAL();
	GDALDatasetH hSrcDS = GDALOpen(lpstrFilePath, GA_ReadOnly);
	if (hSrcDS)
	{
		GDALRasterBandH hBand = GDALGetRasterBand(hSrcDS, 1);
		if (hBand)
		{
			if (MeanVal || StdDev)
			{
				GDALComputeRasterStatistics(hBand, 0, MinVal, MaxVal, MeanVal, StdDev, NULL, NULL);
			}
			else
			{
				double MinMax[2];
				GDALComputeRasterMinMax(hBand, 0, MinMax);
				*MinVal = MinMax[0];
				*MaxVal = MinMax[1];
			}
		}

		GDALClose(hSrcDS);
	}
}

void wInit6Transform(double *pTrasnform)
{
	memset(pTrasnform, 0, 6 * sizeof(double));
	pTrasnform[1] = 1.0;
	pTrasnform[5] = 1.0;
}

void wSetNodataVal(const char *lpstrPath, double dfNodataVal)
{
	GdalUtils::InitGDAL();
	GDALDatasetH pImage = GDALOpen(lpstrPath, GA_Update);
	if (pImage)
	{
		GDALSetRasterNoDataValue(GDALGetRasterBand(pImage, 1), dfNodataVal);
		GDALClose(pImage);
	}
}

const char *wGDALLastError()
{
	return (CPLGetLastErrorMsg());
}
//////////////////////////////////////////////////////////////////////////
void GdalUtils::InitGDAL()
{
	if (!g_isGDALInitialized)
	{
		GDALAllRegister();
		OGRRegisterAll();
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
		g_isGDALInitialized = true;
	}
}

bool GdalUtils::IsGDALInitialized()
{
	return wIsGDALInitialized();
}

int GdalUtils::GDALVersion()
{
	return wGetGDALVersion();
}

void GdalUtils::ConfigGDALOption(const char *szKey, const char *szValue)
{
	wSetGDALConfigOption(szKey, szValue);
}

void GdalUtils::Geo2Raster(const double *pTransform, double *x, double *y)
{
	wGeo2Raster(pTransform, x, y);
}

void GdalUtils::Raster2Geo(const double *pTransform, double *x, double *y)
{
	wRaster2Geo(pTransform, x, y);
}

void GdalUtils::StatisticRaster(const char *lpstrFilePath, double *MinVal, double *MaxVal, double *MeanVal /*=NULL*/, double *StdDev /*=NULL*/)
{
	wStatisticRaster(lpstrFilePath, MinVal, MaxVal, MeanVal, StdDev);
}

void GdalUtils::Init6Transform(double *pTrasnform)
{
	wInit6Transform(pTrasnform);
}

void GdalUtils::Init6GeoTransform(double *pTransform, int h)
{
	memset(pTransform, 0, 6 * sizeof(double));
	pTransform[3] = h;
	pTransform[1] = 1.0;
	pTransform[5] = -1.0;
}

void GdalUtils::InitColorTable(unsigned char *prgb, int dim, int nItemCount /*=256*/)
{
	for (int i = 0; i < nItemCount; ++i)
	{
		for (int j = 0; j < dim; ++j)
		{
			prgb[i * dim + j] = i;
		}
	}
}

void GdalUtils::SetNodataVal(const char *lpstrPath, double dfNodataVal)
{
	wSetNodataVal(lpstrPath, dfNodataVal);
}

const char *GdalUtils::GDALLastError()
{
	return wGDALLastError();
}

bool GdalUtils::GenerateOverviews(const char *lpstrFileFullPath, const char *resampleMethod, int *pLevels, int numLevel)
{
	bool bReadOnly = true;
	GDALDatasetH hDataset;
	GDALProgressFunc pfnProgress = GDALTermProgress;
	const char *pszResampling = resampleMethod;
	const char *pszFilename = lpstrFileFullPath;
	int nBandCount = 0;
	int *panBandList = 0;
	hDataset = GDALOpen(pszFilename, GA_ReadOnly);
	if (hDataset == NULL)
	{
		return false;
	}
	//CPLSetConfigOption("USE_RRD", "YES");
	if (numLevel <= 0)
	{
		return false;
	}

	bool ok = true;
	if (GDALBuildOverviews(hDataset, pszResampling, numLevel, pLevels,
						   nBandCount, panBandList, pfnProgress, NULL) != CE_None)
	{
		ok = false;
	}
	GDALClose(hDataset);
	return ok;
}

bool GdalUtils::GetWidthHeightPixel(const char *lpstrImagePath, int &w, int &h)
{
	GDALDatasetH hDataset = GDALOpen(lpstrImagePath, GA_ReadOnly);

	if (hDataset)
	{
		h = GDALGetRasterYSize(hDataset);
		w = GDALGetRasterXSize(hDataset);
		//	int nBands = GDALGetRasterCount(hDataset);
		GDALClose(hDataset);
		return true;
	}
	return false;
}

bool GdalUtils::CreateAndWriteTiffFloat(const char *tiffFullPath, float *pImageData, int w, int h, double geoTransform[6],
										double invalidDataValue)
{
	InitGDAL();
	float *temp_pixel = pImageData;
	int crow = h;
	int ccol = w;
	GDALDriverH hDriver = GDALGetDriverByName("Gtiff");

	if (hDriver)
	{
		//�����ļ���
		GDALDatasetH hDS = GDALCreate(hDriver, tiffFullPath, ccol, crow, 1, GDT_Float32, NULL);
		if (hDS)
		{
			GDALSetGeoTransform(hDS, geoTransform);

			GDALFlushCache(hDS);
			GDALRasterBandH band = GDALGetRasterBand(hDS, 1);
			GDALSetRasterNoDataValue(band, invalidDataValue);
			CPLErr cplError = GDALRasterIO(band, GF_Write, 0, 0, ccol, crow, temp_pixel, ccol, crow, GDT_Float32, 0, 0);
			if (cplError == CE_Failure)
			{
				return false;
			}
			GDALClose(hDS);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool GdalUtils::CreateAndWriteTiffFloat(float *pData, int w, int h,
										const std::string &tiffFullPath, double noDataValue)
{
	GDALAllRegister();
	OGRRegisterAll();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	const float *temp_pixel = pData;
	int crow = h;
	int ccol = w;
	GDALDriverH hDriver = GDALGetDriverByName("Gtiff");

	if (hDriver)
	{
		//ļ
		GDALDatasetH hDS = GDALCreate(hDriver, tiffFullPath.c_str(), ccol, crow, 1, GDT_Float32, NULL);
		if (hDS)
		{
			GDALFlushCache(hDS);
			GDALRasterBandH band = GDALGetRasterBand(hDS, 1);
			GDALSetRasterNoDataValue(band, noDataValue);
			CPLErr cplError = GDALRasterIO(band, GF_Write, 0, 0, ccol, crow, (void *)temp_pixel, ccol, crow, GDT_Float32, 0, 0);
			if (cplError == CE_Failure)
			{
				return false;
			}
			GDALClose(hDS);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool insight::GdalUtils::ReadTiffFloat(const char *tiffFullPath, std::vector<float> &datas, int &w, int &h, int &depth)
{
	int bands[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	GdalUtils::InitGDAL();
	ImageStream is(tiffFullPath);
	if (!is.IsOpen())
	{
		return false;
	}
	ImageInfo info = is.ImageInformation();
	w = info.Columns();
	h = info.Rows();
	depth = info.Bands();
	datas.resize(w * h * depth);

	is.ReadRange(0, 0, 0, w, h, datas.data(), w, h, w, h, bands, depth);
	return true;
}

int GdalUtils::GetOverviewsCount(const char *lpstrImagePath)
{
	GDALDatasetH hDataset = GDALOpen(lpstrImagePath, GA_ReadOnly);

	if (hDataset)
	{
		int nLevels = GDALGetOverviewCount(GDALGetRasterBand(hDataset, 1));
		GDALClose(hDataset);
		return nLevels;
	}
	return 0;
}

int GdalUtils::Force32bit(int w)
{
	int fillW = w % 4 == 0 ? w : (w / 4 + 1) * 4;
	return fillW;
}

void GdalUtils::Force32bit(std::vector<unsigned char> &rgbImages, int w, int h, int &sw)
{
	int fillW = Force32bit(w);
	if (fillW != sw)
	{
		std::vector<unsigned char> image(fillW * h * 3, 0);
		for (int i = 0; i < h; ++i)
		{
			unsigned char *src = &rgbImages[i * w * 3];
			unsigned char *des = &image[i * fillW * 3];
			memcpy(des, src, w * 3);
		}
		image.swap(rgbImages);
	}
	sw = fillW;
}

static bool resetBuffer(int pixType, int sw, int sh, std::vector<uchar> &buffer)
{
	bool bError = false;
	std::stringstream ss;
	switch (pixType)
	{
	case PIXEL_Unknown:
		ss << "Pixel Unknown " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_Byte:
		buffer.resize(sw * sh * 3, 0);
		break;
	case PIXEL_UInt16:
		buffer.resize(sw * sh * 6, 0);
		break;
	case PIXEL_Int16:
		buffer.resize(sw * sh * 6, 0);
		break;
	case PIXEL_UInt32:
		buffer.resize(sw * sh * 12, 0);
		break;
	case PIXEL_Int32:
		buffer.resize(sw * sh * 12, 0);
		break;
	case PIXEL_Float32:
		buffer.resize(sw * sh * 12, 0);
		break;
	case PIXEL_Float64:
		buffer.resize(sw * sh * 24, 0);
		break;
	case PIXEL_CInt16:
		ss << "Pixel Unknown " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CInt32:
		ss << "PIXEL_CInt32 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CFloat32:
		ss << "PIXEL_CFloat32 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CFloat64:
		ss << "PIXEL_CFloat64 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	default:
		ss << "Unkonw pix type " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	}
	if (bError)
	{
		std::unique_lock<std::mutex> lock(g_gdal_error_mutex);
		g_gdal_error_str = ss.str();
		LOG(ERROR) << g_gdal_error_str << std::endl;
	}
	return !bError;
}
bool GdalUtils::DownScaleReadRGB(const char *lpstrImagePath, int maxDimension, int &w, int &h,
								 int &sw, int &sh, std::vector<uchar> &outRgbImages, int *pix_type)
{
	std::unique_lock<std::mutex> lock(g_gdal_mutex);
	ImageStream stream;
	bool bResult = stream.Open(lpstrImagePath);
	if (!bResult)
	{
		std::cout << "Can't read " << std::string(lpstrImagePath) << std::endl;
		stream.Close();
		return false;
	}

	ImageInfo info = stream.ImageInformation();
	std::map<int, EnColorInterp> mapBandColor = info.MapBandColorInterp();
	std::vector<uchar> buffer;
	EnPixelType pt = info.PixelType();
	if (pix_type != NULL)
		*pix_type = (int)pt;
	std::stringstream ss;
	bool bError = false;

	if (bError)
	{
		std::unique_lock<std::mutex> lock(g_gdal_error_mutex);
		g_gdal_error_str = ss.str();
		LOG(ERROR) << g_gdal_error_str << std::endl;
		return !bError;
	}

	if (info.Bands() < 3)
		return false;
	int iR = -1;
	int iG = -1;
	int iB = -1;
	for (auto itr = mapBandColor.begin(); itr != mapBandColor.end(); ++itr)
	{
		if (itr->second == (int)EnColorInterp::RedBand)
		{
			iR = itr->first;
		}
		else if (itr->second == (int)EnColorInterp::GreenBand)
		{
			iG = itr->first;
		}
		else if (itr->second == (int)EnColorInterp::BlueBand)
		{
			iB = itr->first;
		}
	}

	if (iR < 0 || iG < 0 || iB < 0)
	{
		ss << "Can't find RGB bands ." << __FUNCTION__ << __LINE__;
		bError = true;
	}
	if (bError)
	{
		std::unique_lock<std::mutex> lock(g_gdal_error_mutex);
		g_gdal_error_str = ss.str();
		LOG(ERROR) << g_gdal_error_str << std::endl;
		return !bError;
	}

	w = info.Columns();
	h = info.Rows();
	sw = w, sh = h; //scaled w,h
	bool bRead = false;
	int bands[] = {iR, iG, iB};
	for (int lev = 0; lev < info.Levels(); ++lev)
	{
		int ww = info.ColumnsAt(lev);
		int hh = info.RowsAt(lev);
		if (std::max(ww, hh) > maxDimension)
		{
			continue;
		}
		else
		{
			if (!resetBuffer(pt, ww, hh, buffer))
			{
				return false;
			}
			stream.ReadRange(lev, 0, 0, ww, hh, &buffer[0], ww, hh, ww, hh, bands, 3);
			//std::cout << "Use Level reading...\n";
			sw = ww;
			sh = hh;
			bRead = true;
			break;
		}
	}
	if (!bRead)
	{
		int wh = std::max(w, h);
		while (wh > maxDimension)
		{
			sw = sw >> 1;
			sh = sh >> 1;
			wh = wh >> 1;
		}
		//std::cout << "Use Force Level reading...\n";
		if (!resetBuffer(pt, sw, sh, buffer))
		{
			return false;
		}
		bRead = stream.ReadRange(0, 0, 0, w, h, &buffer[0], sw, sh, sw, sh, bands, 3);
	}

	if (bRead)
		outRgbImages.swap(buffer);
	stream.Close();
	return bRead;
}

void GdalUtils::RgbToGray(const unsigned char *src, int n, unsigned char *des)
{
	for (int i = 0; i < n; ++i)
	{
		des[i] = float(src[i * 3 + 0]) * 0.2126 + float(src[i * 3 + 1]) * 0.7152 + float(src[i * 3 + 2]) * 0.0722;
	}
}


std::string GdalUtils::GetError()
{
	return g_gdal_error_str;
}

bool GdalUtils::RGBForceTo8Bit(EnPixelType pt, const std::vector<uchar> &in, std::vector<uchar> &out)
{
	out.clear();
	if (in.empty())
		return true;
	bool bError = false;
	std::stringstream ss;
	switch (pt)
	{
	case PIXEL_Unknown:
		ss << "Pixel Unknown " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_Byte:
		out = in;
		break;
	case PIXEL_UInt16:
	{
		int s = int(in.size()) / 2;
		out.resize(s, 0);
		const ushort *pSrc = (const ushort *)&in[0];
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			pDes[i] = (int)(float(pSrc[i]) / 65535.f * 255);
		}
	}
	break;
	case PIXEL_Int16:
	{
		int s = int(in.size()) / 2;
		out.resize(s, 0);
		const short *pSrc = (const short *)&in[0];
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			pDes[i] = (int)(float(pSrc[i] + 32768) / 65535.f * 255);
		}
	}
	break;
	case PIXEL_UInt32:
	{
		int s = int(in.size()) / 4;
		out.resize(s, 0);
		const uint *pSrc = (const uint *)&in[0];
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			pDes[i] = (int)(float(pSrc[i]) / 4294967295 * 255);
		}
	}
	break;
	case PIXEL_Int32:
	{
		int s = int(in.size()) / 4;
		out.resize(s, 0);
		const uint *pSrc = (const uint *)&in[0];
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			pDes[i] = (int)(float(pSrc[i] + 2147483648) / 4294967295 * 255);
		}
	}
	break;
	case PIXEL_Float32:
	{
		int s = int(in.size()) / 4;
		out.resize(s, 0);
		const float *pSrc = (const float *)&in[0]; //0-1
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			if (pSrc[i] > 1 || pSrc[i] < 0)
			{
				std::cout << "Error: exceed range [0-1] " << __FUNCTION__ << __LINE__ << std::endl;
			}
			pDes[i] = (int)(pSrc[i] * 255);
		}
	}
	break;
	case PIXEL_Float64:
	{
		int s = int(in.size()) / 8;
		out.resize(s, 0);
		const double *pSrc = (const double *)&in[0]; //0-1
		uchar *pDes = &out[0];
		for (int i = 0; i < s; ++i)
		{
			if (pSrc[i] > 1 || pSrc[i] < 0)
			{
				std::cout << "Error: exceed range [0-1] " << __FUNCTION__ << __LINE__ << std::endl;
			}
			pDes[i] = (int)(pSrc[i] * 255);
		}
	}
	break;
	case PIXEL_CInt16:
		ss << "Pixel Unknown " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CInt32:
		ss << "PIXEL_CInt32 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CFloat32:
		ss << "PIXEL_CFloat32 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	case PIXEL_CFloat64:
		ss << "PIXEL_CFloat64 " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	default:
		ss << "Unkonw pixtype " << __FUNCTION__ << __LINE__;
		bError = true;
		break;
	}
	if (bError)
	{
		std::unique_lock<std::mutex> lock(g_gdal_error_mutex);
		g_gdal_error_str = ss.str();
		LOG(ERROR) << g_gdal_error_str << std::endl;
		return !bError;
	}

	return true;
}

bool GdalUtils::DownScaleReadRGBForce8Bit(const char *lpstrImagePath, int maxDimension, int &w, int &h,
										  int &sw, int &sh, std::vector<uchar> &outRgbImages)
{
	std::vector<uchar> rgbImage;
	int pixType = 0;
	bool OK = DownScaleReadRGB(lpstrImagePath, maxDimension, w, h, sw, sh, rgbImage, &pixType);
	if (!OK)
	{
		return false;
	}
	if (pixType == EnPixelType::PIXEL_Byte)
	{
		rgbImage.swap(outRgbImages);
	}
	else
	{
		OK = RGBForceTo8Bit(EnPixelType(pixType), rgbImage, outRgbImages);
		if (!OK)
		{
			return false;
		}
	}
	return true;
}

}//name space insight
/*#include <gdal/ogr_spatialref.h>*/
