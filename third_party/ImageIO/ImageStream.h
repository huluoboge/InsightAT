#ifndef INSIGHT_IMAGE_STREAM_H
#define INSIGHT_IMAGE_STREAM_H
#include <string>
#include "imageio_global.h"
#include "ImageInfo.h"
namespace insight{

class  ImageStream
{
	typedef void * HImage;
public:
	enum OpenMode{OM_Read = 0,OM_Update = 1};
public:
	enum IMGFormat
	{ 
		IMG_BSQ = 0, // BAND
		IMG_BIP = 1, // PIXEL
		IMG_BIL = 2 // LINE
	};
public:
	static  bool Create(const char* lpstrFilePath,int nCol,int nRow,int nBand,double* dTransform, 
		EnPixelType pixType=PIXEL_Byte,const char* strCode="GTiff",const char* projWKT=NULL, 
		bool bTiled=false,bool bCompress=false,const char* strTieSize="128",const char* strCompress="LZW");	
public:
	ImageStream();
	~ImageStream();

	void setComputeMinMax(bool compute);
	ImageStream(const std::string &filePath, OpenMode enumOpenModes = OM_Read);
	
	bool Open(const std::string &filePath, OpenMode nOpenMode = OM_Read);


	bool SimpleOpen(const std::string &filePath, OpenMode openMode = OM_Read);

	bool SimpleInfomation(int &width, int &hight, int &bandCount);
	
	bool IsOpen() const;

    /**
     * @brief �ر���
     */
	void Close();

	bool ReadRange (int nLevel,int nFileSX,int nFileSY,int nFileXSize,int nFileYSize,
		void *pBuffer,int nReadXSize,int nReadYSize,int nBufXSize,int nBufYSize,
		int *pBandList,int nBandNum,IMGFormat eF=IMG_BIP);

	bool WriteRange(int nLevel,int nFileSX,int nFileSY,int nFileXSize,int nFileYSize,
		void *pBuffer,int nWriteXSize,int nWriteYSize,int nBufXSize,int nBufYSize,
		int *pBandList,int nBandNum,IMGFormat eF=IMG_BIP);
	ImageInfo ImageInformation() const;

public:
	std::string FilePath() const{return m_strFilePath;}
private:
	void Init();
private:
	static bool m_bInitialized;//false
	bool m_bOpen;
	std::string m_strFilePath;
	OpenMode m_enumOpenMode;
	ImageInfo m_imageInfomation;
	bool m_computeMinMax;//false;
};
}//name space insight
#endif
