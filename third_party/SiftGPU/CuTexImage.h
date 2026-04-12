////////////////////////////////////////////////////////////////////////////
//	File:		CuTexImage.h
//	Author:		Changchang Wu
//	Description :	interface for the CuTexImage class.
//					class for storing data in CUDA.
//
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty.
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////

#ifndef CU_TEX_IMAGE_H
#define CU_TEX_IMAGE_H

class GLTexImage;
struct cudaArray;

// Use texture2D from linear memory (pitch2D resource)
#define SIFTGPU_ENABLE_LINEAR_TEX2D

class CuTexImage
{
protected:
	void*		_cuData;
	cudaArray*	_cuData2D;
	int			_numChannel;
	size_t		_numBytes;
	int			_imgWidth;
	int			_imgHeight;
	int			_texWidth;
	int			_texHeight;
	GLuint		_fromPBO;
public:
	virtual void SetImageSize(int width, int height);
	virtual bool InitTexture(int width, int height, int nchannel = 1);
	void InitTexture2D();
	void CopyToTexture2D();
	void CopyToHost(void* buf);
	void CopyToHost(void* buf, int stream);
	void CopyFromHost(const void* buf);
	int  CopyToPBO(GLuint pbo);
	void CopyFromPBO(int width, int height, GLuint pbo);
	static int DebugCopyToTexture2D();
public:
	inline int GetImgWidth(){return _imgWidth;}
	inline int GetImgHeight(){return _imgHeight;}
	inline int GetDataSize(){return _numBytes;}
// Texture object methods are only available when CUDA runtime headers are included.
// In .cu files __CUDACC__ is defined; in .cpp files that include <cuda_runtime_api.h>
// the type cudaTextureObject_t is available via __CUDA_RUNTIME_H__.
#if defined(__CUDACC__) || defined(__CUDA_RUNTIME_H__)
	// Create a cuda texture object for the current image buffer (1D linear).
	inline cudaTextureObject_t CreateTextureObject();
	// Create a 2D texture object (for tex2D<> in kernels). Uses pitch2D or CUDA array.
	inline cudaTextureObject_t CreateTextureObject2D();
	inline void DestroyTextureObject(cudaTextureObject_t obj);
#endif
public:
	CuTexImage();
	CuTexImage(int width, int height, int nchannel, GLuint pbo);
	virtual ~CuTexImage();
	friend class ProgramCU;
	friend class PyramidCU;
};

#endif // !defined(CU_TEX_IMAGE_H)
