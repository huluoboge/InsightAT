#ifndef INSIGNT_IMAGE_INFO_H
#define INSIGNT_IMAGE_INFO_H
#include "imageio_global.h"

#include <map>
#include <vector>
namespace insight{
enum EnPixelType {
    PIXEL_Unknown = 0, /*! Unknown or unspecified type */
    PIXEL_Byte = 1, /*! Eight bit unsigned integer */
    PIXEL_UInt16 = 2, /*! Sixteen bit unsigned integer */
    PIXEL_Int16 = 3, /*! Sixteen bit signed integer */
    PIXEL_UInt32 = 4, /*! Thirty two bit unsigned integer */
    PIXEL_Int32 = 5, /*! Thirty two bit signed integer */
    PIXEL_Float32 = 6, /*! Thirty two bit floating point */
    PIXEL_Float64 = 7, /*! Sixty four bit floating point */
    PIXEL_CInt16 = 8, /*! Complex Int16 */
    PIXEL_CInt32 = 9, /*! Complex Int32 */
    PIXEL_CFloat32 = 10, /*! Complex Float32 */
    PIXEL_CFloat64 = 11, /*! Complex Float64 */
};

enum EnColorInterp // same with GDALColorInterp
{
    Undefined = 0,
    /*! Greyscale */ GrayIndex = 1,
    /*! Paletted (see associated color table) */ PaletteIndex = 2,
    /*! Red band of RGBA image */ RedBand = 3,
    /*! Green band of RGBA image */ GreenBand = 4,
    /*! Blue band of RGBA image */ BlueBand = 5,
    /*! Alpha (0=transparent, 255=opaque) */ AlphaBand = 6,
    /*! Hue band of HLS image */ HueBand = 7,
    /*! Saturation band of HLS image */ SaturationBand = 8,
    /*! Lightness band of HLS image */ LightnessBand = 9,
    /*! Cyan band of CMYK image */ CyanBand = 10,
    /*! Magenta band of CMYK image */ MagentaBand = 11,
    /*! Yellow band of CMYK image */ YellowBand = 12,
    /*! Black band of CMLY image */ BlackBand = 13,
    /*! Y Luminance */ YCbCr_YBand = 14,
    /*! Cb Chroma */ YCbCr_CbBand = 15,
    /*! Cr Chroma */ YCbCr_CrBand = 16,
    /*! Max current value */ Max = 16
};

class ImageStream;
class  ImageInfo {
    friend class ImageStream;
    typedef void* HImage;

public:
    explicit ImageInfo();

    void Reset();

    int Columns() const { return m_nColumns; }

    int Rows() const { return m_nRows; }

    int Bands() const { return m_nBands; }

    int PixelBits() const { return m_nPixelBits; }

    EnPixelType PixelType() const { return m_enumPixelType; }
    int XTileSize() const { return m_nBlockXSize; }
    int YTileSize() const { return m_nBlockYSize; }
    int TileColumns() const;
    int TileRows() const;

    int Levels() const { return m_nLevels; }

    int ColumnsAt(int nLevel) const;

    int RowsAt(int nLevel) const;

    double MinValue() const { return m_dMinValue; }
    double MaxValue() const { return m_dMaxValue; }

    double NoDataValue() const { return m_dNoDataVal; }

    void GetGeoTransForm(double* geoTransfrom) const;

    bool IsGeoTransformValid() const { return m_geoTransformValid; }
    std::string Projection() const;

    int PixelBytes() const;

    bool IsValid() const { return m_nColumns > 0 && m_nRows > 0; }
    bool IsTiled() const { return (m_nBlockXSize != m_nColumns && m_nBlockYSize != 1); }

    bool HasNoDataValue() const { return m_bHasNoDataVal; }
    std::map<int, EnColorInterp> MapBandColorInterp() const { return m_mapBandColorInterp; }

private:
    int m_nColumns;
    int m_nRows;
    int m_nBands;
    int m_nPixelBits;
    int m_nBlockXSize; // 0
    int m_nBlockYSize;
    int m_nLevels; // 1
    double m_dMinValue; // 0.0
    double m_dMaxValue; // 0.0
    EnPixelType m_enumPixelType;
    HImage m_hImage;
    ImageStream* m_pImageStream; // NULL
    unsigned char m_rgbaTable[256 * 4];
    double m_geoTransform[6];
    bool m_geoTransformValid; // false;
    std::string m_strProjection;
    double m_dNoDataVal; //
    bool m_bHasNoDataVal; // false
    std::vector<int> m_overviewRows;
    std::vector<int> m_overviewColumns;
    std::map<int, EnColorInterp> m_mapBandColorInterp;
};
}//name space insight
#endif
