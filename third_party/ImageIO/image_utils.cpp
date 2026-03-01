#include "image_utils.h"

namespace insight{


void image_utils::makeBlocks(int width, int height, 
int blockSize, std::vector<ImageBlock> &blocks, 
const int edgePixel /*= 10*/,
const int band)
{
	int maxBlockSize = blockSize;
	std::vector<int> vec_widths;
	std::vector<int> vec_height;
	vec_widths.resize(width / maxBlockSize + (width % maxBlockSize == 0 ? 0 : 1), maxBlockSize + edgePixel);
	vec_height.resize(height / maxBlockSize + (height % maxBlockSize == 0 ? 0 : 1), maxBlockSize + edgePixel);
	//ƽ����
	int block_w = width / vec_widths.size();
	int block_h = height / vec_height.size();
	for (int i = 0; i < vec_widths.size(); ++i){
		vec_widths[i] = block_w + edgePixel;
	}
	for (int i = 0; i < vec_height.size(); ++i)
	{
		vec_height[i] = block_h + edgePixel;
	}
	//vec_widths.back() = width - (maxBlockSize * (width / maxBlockSize));
	//if (vec_widths.back() == 0) vec_widths.back() = maxBlockSize;
//	vec_height.back() = height - (maxBlockSize * (height / maxBlockSize));
	//if (vec_height.back() == 0) vec_height.back() = maxBlockSize;
	blocks.resize(vec_widths.size() * vec_height.size());
	for (int row = 0; row < vec_height.size(); ++row)
	{
		for (int col = 0; col < vec_widths.size(); ++col)
		{
			ImageBlock &block = blocks[row * vec_widths.size() + col];
			block.col= block_w * col;
			block.row= block_h * row;
			block.width = vec_widths[col];
			block.height = vec_height[row];
			if (block.col + block.width >= width){
				block.width = width - block.col;
			}
			if (block.row + block.height >= height)
			{
				block.height = height - block.row;
			}
			block.vec_imageData.resize(band * block.width * block.height, 0);
		}
	}
}



void image_utils::copyToBlocks(int width, int height, 
	const std::vector<unsigned char> &src_vec_imageData, 
	std::vector<ImageBlock> &desBlocks,
	const int band)
{
	for (int ib = 0; ib < desBlocks.size(); ++ib)
	{
		ImageBlock &block = desBlocks[ib];
		if (block.vec_imageData.empty()) continue;

		const unsigned char *pData = &src_vec_imageData[band * (width * block.row + block.col)];//rgb,3bit
		for (int r = 0; r < block.height; ++r)
		{
			const unsigned char *pRow = &pData[r * width * band];
			unsigned char *pDesRow = &block.vec_imageData[r * block.width * band];
			for (int c = 0; c < block.width * band; ++c)
			{
				pDesRow[c] = pRow[c];
			}
		}
	}
}

















}//name space insight