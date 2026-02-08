#ifndef image_utils_h__
#define image_utils_h__

#include "imageio_global.h"
#include <vector>


namespace insight{

struct ImageBlock
{
	int row = 0;
	int col = 0;
	int width = 0;
	int height = 0;
	std::vector<unsigned char> vec_imageData;
};

class  image_utils
{
public:
	static void makeBlocks(int width, int height, 
		int blockSize, std::vector<ImageBlock> &blocks, 
		const int edgePixel /*= 10*/,
		const int band = 3);

	static void copyToBlocks(int width, int height, 
		const std::vector<unsigned char> &src_vec_imageData, 
		std::vector<ImageBlock> &desBlocks,
		const int band = 3);

};
}//name space insight
#endif // image_utils_h__
