#include "ImageStream.h"
#include "ImageInfo.h"
#include "cpl_string.h"
#include "gdal.h"
#include "gdal_utils.h"
#include "ogr_api.h"
#include <cstdint>
#include <iostream>

namespace insight {

ImageStream::ImageStream()
{
    Init();
}

ImageStream::ImageStream(const std::string& filePath, OpenMode enumOpenModes /*= OM_Read*/)
{
    Init();
    Open(filePath, enumOpenModes);
}

ImageStream::~ImageStream()
{
    if (IsOpen()) {
        Close();
    }
}
void ImageStream::Init()
{
    GdalUtils::InitGDAL();
    m_bOpen = false;
    m_enumOpenMode = OM_Read;
    m_computeMinMax = false;
}

bool ImageStream::Open(const std::string& filePath, OpenMode nOpenMode /*= modeREAD*/)
{
    if (IsOpen()) {
        Close();
    }
    m_strFilePath = filePath;
    m_imageInfomation.Reset();
    m_imageInfomation.m_hImage = GDALOpen(filePath.c_str(), nOpenMode == OM_Read ? GA_ReadOnly : GA_Update);

    if (m_imageInfomation.m_hImage) {
        m_enumOpenMode = nOpenMode;
        const char* pszProjectionRef = GDALGetProjectionRef(m_imageInfomation.m_hImage);
        if (pszProjectionRef) {
            m_imageInfomation.m_strProjection = pszProjectionRef;
        }

        int nCols = GDALGetRasterXSize(m_imageInfomation.m_hImage);
        int nRows = GDALGetRasterYSize(m_imageInfomation.m_hImage);
        int nBands = GDALGetRasterCount(m_imageInfomation.m_hImage);

        CPLErr error = GDALGetGeoTransform(m_imageInfomation.m_hImage, m_imageInfomation.m_geoTransform);

        if (error != CPLErr::CE_None) {
            m_imageInfomation.m_geoTransformValid = false;
        } else {
            m_imageInfomation.m_geoTransformValid = true;
        }

        for (int i = 1; i <= nBands; ++i) {
            GDALRasterBandH hBand = GDALGetRasterBand(m_imageInfomation.m_hImage, i);
            m_imageInfomation.m_mapBandColorInterp[i] = (EnColorInterp)(int)(GDALGetRasterColorInterpretation(hBand));
        }

        GDALDataType dataType = GDALGetRasterDataType(GDALGetRasterBand(m_imageInfomation.m_hImage, 1));
        int nBits = GDALGetDataTypeSize(dataType);
        EnPixelType enumPixelType = (EnPixelType)dataType;
        m_imageInfomation.m_nColumns = nCols;
        m_imageInfomation.m_nRows = nRows;
        m_imageInfomation.m_nBands = nBands;
        m_imageInfomation.m_nPixelBits = nBits;
        m_imageInfomation.m_enumPixelType = enumPixelType;

        // ��ɫ��;
        GDALColorTableH colorTable = GDALGetRasterColorTable(GDALGetRasterBand(m_imageInfomation.m_hImage, 1));
        if (colorTable != NULL) {
            int nCount = GDALGetColorEntryCount(colorTable);
            for (int i = 0; i < nCount && i < 256; ++i) {
                const GDALColorEntry* psEntry = GDALGetColorEntry(colorTable, i);
                // TODO:Ӧ��֧�� cmyk,hsv ģ�� by Jones.
                m_imageInfomation.m_rgbaTable[i * 4] = (uchar)psEntry->c1;
                m_imageInfomation.m_rgbaTable[i * 4 + 1] = (uchar)psEntry->c2;
                m_imageInfomation.m_rgbaTable[i * 4 + 2] = (uchar)psEntry->c3;
                m_imageInfomation.m_rgbaTable[i * 4 + 3] = (uchar)psEntry->c4;
            }
        }
        int nBlockXSize;
        int nBlockYSize;
        int nLevels;
        int bHasNoDataValue = 0;
        GDALGetBlockSize(GDALGetRasterBand(m_imageInfomation.m_hImage, 1), &nBlockXSize, &nBlockYSize);

        nLevels = GDALGetOverviewCount(GDALGetRasterBand(m_imageInfomation.m_hImage, 1));
        GDALRasterBandH hBand = GDALGetRasterBand(m_imageInfomation.m_hImage, 1);
        if (hBand) {
            for (int i = 0; i < nLevels; ++i) {
                GDALRasterBandH hBand1 = GDALGetOverview(hBand, i);
                if (hBand1) {
                    m_imageInfomation.m_overviewColumns.push_back(GDALGetRasterBandXSize(hBand1));
                    m_imageInfomation.m_overviewRows.push_back(GDALGetRasterBandYSize(hBand1));
                }
            }
        }
        m_imageInfomation.m_nLevels += nLevels;
        m_imageInfomation.m_dNoDataVal = GDALGetRasterNoDataValue(hBand, &bHasNoDataValue);

        if (bHasNoDataValue) {
            m_imageInfomation.m_bHasNoDataVal = true;
        } else {
            m_imageInfomation.m_bHasNoDataVal = false;
        }

        m_imageInfomation.m_nBlockXSize = nBlockXSize;
        m_imageInfomation.m_nBlockYSize = nBlockYSize;

        if (m_computeMinMax) {
            double dMinMax[2];
            GDALComputeRasterMinMax(hBand, 0, dMinMax);
            m_imageInfomation.m_dMinValue = dMinMax[0];
            m_imageInfomation.m_dMaxValue = dMinMax[1];
        }

        m_bOpen = true;
    } else {
        m_bOpen = false;
    }
    return m_bOpen;
}

bool ImageStream::IsOpen() const
{
    return m_bOpen;
}

void ImageStream::Close()
{
    if (m_imageInfomation.m_hImage) {
        GDALClose(m_imageInfomation.m_hImage);
    }
    m_imageInfomation.Reset();
    m_bOpen = false;
    m_strFilePath.clear();
}

bool ImageStream::ReadRange(int nLevel, int nFileSX, int nFileSY, int nFileXSize, int nFileYSize,
    void* pBuffer, int nReadXSize, int nReadYSize, int nBufXSize,
    int nBufYSize, int* pBandList, int nBandNum, IMGFormat eF /*=BIP*/)
{
    if (!IsOpen()) {
        return false;
    }
    if (nFileXSize <= 0 || nFileYSize <= 0 || nReadXSize <= 0 || nReadYSize <= 0) {
        return false;
    }
    if (nLevel >= m_imageInfomation.m_nLevels) {
        return false;
    }
    // ÿһ�����ε�����Ӧ����һ�µ�
    GDALDataType eType = GDALGetRasterDataType(GDALGetRasterBand(m_imageInfomation.m_hImage, 1));
    int nBands = GDALGetRasterCount(m_imageInfomation.m_hImage);
    if (nBands < nBandNum) {
        std::cout << "Error : exceed bands counts! " << __FUNCTION__ << __LINE__ << std::endl;
        return false;
    }
    int nPixelSpace, nLineSpace, nBandSpace;
    /*
    BSQ
    rrrrrrrrr
    rrrrrrrrr
    ggggggggg
    ggggggggg
    bbbbbbbbb
    bbbbbbbbb
    BIL
    rrrrrrrrr
    ggggggggg
    bbbbbbbbb
    rrrrrrrrr
    ggggggggg
    bbbbbbbbb
    BIP
    rgbrgbrgb
    rgbrgbrgb
    rgbrgbrgb
    rgbrgbrgb
    rgbrgbrgb
    rgbrgbrgb
    */
    switch (eF) {
    case IMG_BSQ: // BAND
        nPixelSpace = GDALGetDataTypeSize(eType) / 8;
        nLineSpace = nPixelSpace * nBufXSize;
        nBandSpace = nLineSpace * nBufYSize;
        printf("IMG_BSQ\n");
        break;
    case IMG_BIP: // PIXEL
        nPixelSpace = GDALGetDataTypeSize(eType) / 8 * nBandNum;
        nLineSpace = nPixelSpace * nBufXSize;
        nBandSpace = GDALGetDataTypeSize(eType) / 8;
        printf("IMG_BIP\n");
        break;
    case IMG_BIL: // LINE;
        nPixelSpace = GDALGetDataTypeSize(eType) / 8;
        nLineSpace = nPixelSpace * nBufXSize * nBandNum;
        nBandSpace = nPixelSpace * nBufXSize;
        printf("IMG_BIL\n");
        break;
    default:
        throw std::runtime_error("unkown gdal data type");
    };

    // printf("pixel=%d,line=%d,band=%d\n",nPixelSpace,nLineSpace,nBandSpace);

    GDALRasterBandH hBand;
    for (int iBand = 0; iBand < nBandNum; iBand++) {
        hBand = GDALGetRasterBand(m_imageInfomation.m_hImage, pBandList[iBand]);
        if (!hBand) {
            continue;
        }
        if (nLevel > 0) {
            hBand = GDALGetOverview(hBand, nLevel - 1);
        }
        CPLErr cplError = GDALRasterIO(hBand, GF_Read, nFileSX, nFileSY, nFileXSize, nFileYSize,
            (uint8_t*)pBuffer + nBandSpace * iBand, nReadXSize, nReadYSize, eType, nPixelSpace, nLineSpace);
        if (cplError == CE_Failure) {
            return false;
        }
    }

    return true;
}

bool ImageStream::WriteRange(int nLevel, int nFileSX, int nFileSY, int nFileXSize, int nFileYSize,
    void* pBuffer, int nWriteXSize, int nWriteYSize, int nBufXSize, int nBufYSize,
    int* pBandList, int nBandNum, IMGFormat eF /*=BIP*/)
{
    if (!IsOpen() || m_enumOpenMode != OM_Update) {
        return false;
    }
    if (nFileXSize <= 0 || nFileYSize <= 0 || nWriteXSize <= 0 || nWriteYSize <= 0) {
        return false;
    }
    if (nLevel >= m_imageInfomation.m_nLevels) {
        return false;
    }

    GDALDataType eType = GDALGetRasterDataType(GDALGetRasterBand(m_imageInfomation.m_hImage, 1));
    int nPixelSpace, nLineSpace, nBandSpace;
    switch (eF) {
    case IMG_BSQ: // BAND
        nPixelSpace = GDALGetDataTypeSize(eType) / 8;
        nLineSpace = nPixelSpace * nBufXSize;
        nBandSpace = nLineSpace * nBufYSize;
        break;
    case IMG_BIP: // PIXEL
        nPixelSpace = GDALGetDataTypeSize(eType) / 8 * nBandNum;
        nLineSpace = nPixelSpace * nBufXSize;
        nBandSpace = GDALGetDataTypeSize(eType) / 8;
        break;
    case IMG_BIL: // LINE;
        nPixelSpace = GDALGetDataTypeSize(eType) / 8;
        nLineSpace = nPixelSpace * nBufXSize * nBandNum;
        nBandSpace = nPixelSpace * nBufXSize;
        break;
    };

    GDALRasterBandH hBand;
    for (int iBand = 0; iBand < nBandNum; iBand++) {
        hBand = GDALGetRasterBand(m_imageInfomation.m_hImage, pBandList[iBand]);
        if (!hBand) {
            continue;
        }
        if (nLevel > 0) {
            hBand = GDALGetOverview(hBand, nLevel - 1);
        }

        CPLErr cplError = GDALRasterIO(hBand, GF_Write, nFileSX, nFileSY, nFileXSize, nFileYSize, (uint8_t*)pBuffer + nBandSpace * iBand, nWriteXSize, nWriteYSize, eType, nPixelSpace, nLineSpace);
        if (cplError == CE_Failure) {
            return false;
        }
    }

    return true;
}

ImageInfo ImageStream::ImageInformation() const
{
    return m_imageInfomation;
}

bool ImageStream::Create(const char* lpstrFilePath, int nCol, int nRow, int nBand, double* dTransform,
    EnPixelType pixType /*=PIXEL_Byte*/, const char* strCode /*="GTiff"*/,
    const char* projWKT /*=NULL*/, bool bTiled /*=false*/, bool bCompress /*=false*/,
    const char* strTieSize /*="128"*/, const char* strCompress /*="LZW"*/)
{
    bool bSuccess = false;
    if (dTransform == NULL) {
        return false;
    }

    GDALDriverH hDriver = GDALGetDriverByName(strCode);

    if (hDriver) {
        char** papszMetadata = GDALGetMetadata(hDriver, NULL); //
        if (!CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE)) {
            printf("Driver %s NOT supports Create() method.\n", strCode);
            return false;
        }

        GDALDataType eType = (GDALDataType)pixType;

        char** ppStr = NULL;
        if (hDriver == GDALGetDriverByName("GTiff")) {
            if (bTiled) {
                ppStr = CSLAddNameValue(ppStr, "TILED", "TRUE");
                ppStr = CSLAddNameValue(ppStr, "BLOCKXSIZE", strTieSize);
                ppStr = CSLAddNameValue(ppStr, "BLOCKYSIZE", strTieSize);
            }
            if (bCompress) {
                ppStr = CSLAddNameValue(ppStr, "COMPRESS", strCompress);
            }
        } else if (hDriver == GDALGetDriverByName("HFA")) {
            if (bTiled) {
                ppStr = CSLAddNameValue(ppStr, "BLOCKSIZE", strTieSize);
            }
            if (bCompress) {
                ppStr = CSLAddNameValue(ppStr, "COMPRESSED", "TRUE");
            }
        }

        GDALDatasetH hDS = GDALCreate(hDriver, lpstrFilePath, nCol, nRow, nBand, eType, ppStr);
        if (hDS) {
            GDALSetGeoTransform(hDS, dTransform);
            if (projWKT)
                GDALSetProjection(hDS, projWKT);
            GDALFlushCache(hDS);
            bSuccess = true;
        }

        GDALClose(hDS);

    } else {
        printf("code not exist\n");
    }
    return bSuccess;
}

void ImageStream::setComputeMinMax(bool compute)
{
    m_computeMinMax = compute;
}

bool ImageStream::SimpleOpen(const std::string& filePath, OpenMode nOpenMode /*= OM_Read*/)
{
    if (IsOpen()) {
        Close();
    }
    m_strFilePath = filePath;
    m_imageInfomation.Reset();
    m_imageInfomation.m_hImage = GDALOpen(filePath.c_str(), nOpenMode == OM_Read ? GA_ReadOnly : GA_Update);

    if (!m_imageInfomation.m_hImage) {
        m_bOpen = false;
    } else {
        m_enumOpenMode = nOpenMode;
        m_bOpen = true;
    }
    return m_bOpen;
}

bool ImageStream::SimpleInfomation(int& width, int& hight, int& bandCount)
{
    if (!IsOpen()) {
        return false;
    }
    width = GDALGetRasterXSize(m_imageInfomation.m_hImage);
    hight = GDALGetRasterYSize(m_imageInfomation.m_hImage);
    bandCount = GDALGetRasterCount(m_imageInfomation.m_hImage);
    return true;
}

} // name space insight
