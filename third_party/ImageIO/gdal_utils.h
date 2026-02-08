/************************************************************************/
/*
@Author: Jones
@Date: 2018-02-14

*/
/************************************************************************/
#ifndef INSIGHT_IMAGE_IO_H
#define INSIGHT_IMAGE_IO_H
#include "ImageInfo.h"
#include "imageio_global.h"
#include <string>
#include <vector>

namespace insight{

class  GdalUtils {
public:
    static void InitGDAL();

    static void DestoryGDAL();

    static void SetDataPath(const std::string& path);
    static bool IsGDALInitialized();

    static int GDALVersion();

    static void ConfigGDALOption(const char* szKey, const char* szValue);

    static void Geo2Raster(const double* pTransform, double* x, double* y);

    static void Raster2Geo(const double* pTransform, double* x, double* y);

    static void StatisticRaster(const char* lpstrFilePath, double* MinVal, double* MaxVal, double* MeanVal = NULL, double* StdDev = NULL);

    static void Init6Transform(double* pTrasnform);

    static void Init6GeoTransform(double* pTransform, int h);
    static void InitColorTable(unsigned char* pRgb, int dim, int nItemCount = 256);

    static void SetNodataVal(const char* lpstrPath, double dfNodataVal);

    static const char* GDALLastError();

    static bool GetWidthHeightPixel(const char* lpstrImagePath, int& w, int& h);

    static bool GenerateOverviews(const char* lpstrFileFullPath, const char* resampleMethod, int* pLevels, int numLevel);

    static bool CreateAndWriteTiffFloat(const char* tiffFullPath, float* pImageData, int w, int h, double geoTransform[6], double noDataValue = -9999);

    static bool CreateAndWriteTiffFloat(float* pData, int w, int h,
        const std::string& tiffFullPath, double noDataValue = -9999);

    static bool ReadTiffFloat(const char* tiffFullPath, std::vector<float>& datas, int& w, int& h, int& depth);
    static int GetOverviewsCount(const char* tiffFullPath);

    static int Force32bit(int w);

    static void Force32bit(std::vector<unsigned char>& rgbImages, int w, int h, int& fillW);
   
    static bool RGBForceTo8Bit(EnPixelType pt, const std::vector<uchar>& in, std::vector<uchar>& out);

    static bool DownScaleReadRGBForce8Bit(const char* lpstrImagePath, int maxDimension,
        int& w, int& h, int& sw, int& sh, std::vector<uchar>& outRgbImages);

    static bool DownScaleReadRGB(const char* lpstrImagePath, int maxDimension, int& w, int& h,
        int& sw, int& sh, std::vector<uchar>& outRgbImages, int* pix_type = NULL);

    // 3 band color rgb to 1 band gray
    static void RgbToGray(const unsigned char* src, int n, unsigned char* des);

    static std::string GetError();

private:
    GdalUtils() { }
};
}//name space insight
#endif
