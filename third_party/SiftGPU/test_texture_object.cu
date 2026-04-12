/**
 * test_texture_object.cu
 * Unit test for CuTexImage texture object API (CUDA 12.x migration validation).
 *
 * Tests that CreateTextureObject() and CreateTextureObjectUint() produce
 * correct results when read via tex1Dfetch<float>, tex1Dfetch<float4>,
 * tex1Dfetch<uint4>, and tex1Dfetch<int4> in kernels.
 *
 * Build:
 *   nvcc -std=c++17 test_texture_object.cu -o test_texture_object -I. -lGL -lGLEW -lEGL
 *
 * Or via CMake target test_siftgpu_texture (see SiftGPU/CMakeLists.txt).
 */

#if defined(CUDA_SIFTGPU_ENABLED)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <cuda_runtime.h>

#include "GL/glew.h"
#include "CuTexImage.h"
#include "ProgramCU.h"
#include "GlobalUtil.h"

// Provide the texture object methods (defined as inline in ProgramCU.cu,
// not exported from the library). These must match ProgramCU.cu exactly.

cudaTextureObject_t CuTexImage::CreateTextureObject()
{
	cudaResourceDesc resDesc; memset(&resDesc, 0, sizeof(resDesc));
	cudaChannelFormatDesc chDesc;
	if(_numChannel == 1) chDesc = cudaCreateChannelDesc(32,0,0,0,cudaChannelFormatKindFloat);
	else if(_numChannel == 2) chDesc = cudaCreateChannelDesc(32,32,0,0,cudaChannelFormatKindFloat);
	else if(_numChannel == 4) chDesc = cudaCreateChannelDesc(32,32,32,32,cudaChannelFormatKindFloat);
	else chDesc = cudaCreateChannelDesc(32,0,0,0,cudaChannelFormatKindFloat);
	resDesc.resType = cudaResourceTypeLinear;
	resDesc.res.linear.devPtr = _cuData;
	resDesc.res.linear.desc = chDesc;
	resDesc.res.linear.sizeInBytes = _numBytes;
	cudaTextureDesc texDesc; memset(&texDesc, 0, sizeof(texDesc));
	texDesc.filterMode = cudaFilterModePoint;
	texDesc.readMode = cudaReadModeElementType;
	cudaTextureObject_t obj = 0;
	cudaCreateTextureObject(&obj, &resDesc, &texDesc, NULL);
	return obj;
}

cudaTextureObject_t CuTexImage::CreateTextureObjectUint()
{
	cudaResourceDesc resDesc; memset(&resDesc, 0, sizeof(resDesc));
	cudaChannelFormatDesc chDesc;
	if(_numChannel == 1) chDesc = cudaCreateChannelDesc(32,0,0,0,cudaChannelFormatKindUnsigned);
	else if(_numChannel == 2) chDesc = cudaCreateChannelDesc(32,32,0,0,cudaChannelFormatKindUnsigned);
	else if(_numChannel == 4) chDesc = cudaCreateChannelDesc(32,32,32,32,cudaChannelFormatKindUnsigned);
	else chDesc = cudaCreateChannelDesc(32,0,0,0,cudaChannelFormatKindUnsigned);
	resDesc.resType = cudaResourceTypeLinear;
	resDesc.res.linear.devPtr = _cuData;
	resDesc.res.linear.desc = chDesc;
	resDesc.res.linear.sizeInBytes = _numBytes;
	cudaTextureDesc texDesc; memset(&texDesc, 0, sizeof(texDesc));
	texDesc.filterMode = cudaFilterModePoint;
	texDesc.readMode = cudaReadModeElementType;
	cudaTextureObject_t obj = 0;
	cudaCreateTextureObject(&obj, &resDesc, &texDesc, NULL);
	return obj;
}

cudaTextureObject_t CuTexImage::CreateTextureObject2D() { return 0; }
void CuTexImage::DestroyTextureObject(cudaTextureObject_t obj) { if(obj) cudaDestroyTextureObject(obj); }

// Stub for ProgramCU::CheckErrorCUDA (referenced by CuTexImage.cpp)
int ProgramCU::CheckErrorCUDA(const char* location) {
	cudaError_t e = cudaGetLastError();
	if(e) { if(location) fprintf(stderr, "%s: %s\n", location, cudaGetErrorString(e)); return 1; }
	return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test kernels
// ─────────────────────────────────────────────────────────────────────────────

/// Read N floats via tex1Dfetch<float> from a 1-channel texture object.
__global__ void read_float_kernel(float* out, int n, cudaTextureObject_t tex) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) out[i] = tex1Dfetch<float>(tex, i);
}

/// Read N float4 via tex1Dfetch<float4> from a 4-channel float texture object.
__global__ void read_float4_kernel(float4* out, int n, cudaTextureObject_t tex) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) out[i] = tex1Dfetch<float4>(tex, i);
}

/// Read N uint4 via tex1Dfetch<uint4> from a 4-channel uint texture object.
__global__ void read_uint4_kernel(uint4* out, int n, cudaTextureObject_t tex) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) out[i] = tex1Dfetch<uint4>(tex, i);
}

/// Read N int4 via tex1Dfetch<int4> from a 4-channel uint texture object.
__global__ void read_int4_kernel(int4* out, int n, cudaTextureObject_t tex) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) out[i] = tex1Dfetch<int4>(tex, i);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { fprintf(stderr, "FAIL: %s\n", msg); g_fail++; } \
  else { g_pass++; } \
} while(0)

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: float texture (1 channel) — used by FilterH, FilterV, DOG, etc.
// ─────────────────────────────────────────────────────────────────────────────
static void test_float_texture() {
  printf("--- Test: float texture (1 channel) ---\n");
  const int N = 256;
  std::vector<float> host_data(N);
  for (int i = 0; i < N; i++) host_data[i] = (float)i * 0.1f + 1.0f;

  CuTexImage img;
  img.InitTexture(N, 1, 1);
  img.CopyFromHost(host_data.data());

  cudaTextureObject_t tex = img.CreateTextureObject();

  float* d_out;
  cudaMalloc(&d_out, N * sizeof(float));
  read_float_kernel<<<(N+63)/64, 64>>>(d_out, N, tex);
  cudaDeviceSynchronize();

  std::vector<float> result(N);
  cudaMemcpy(result.data(), d_out, N * sizeof(float), cudaMemcpyDeviceToHost);

  int errors = 0;
  for (int i = 0; i < N; i++) {
    if (fabsf(result[i] - host_data[i]) > 1e-6f) {
      if (errors < 5)
        fprintf(stderr, "  [%d] expected %.6f, got %.6f\n", i, host_data[i], result[i]);
      errors++;
    }
  }
  CHECK(errors == 0, "float texture readback");
  printf("  float: %d/%d correct\n", N - errors, N);

  img.DestroyTextureObject(tex);
  cudaFree(d_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: float4 texture (4 channels) — used by keypoints, descriptors (norm)
// ─────────────────────────────────────────────────────────────────────────────
static void test_float4_texture() {
  printf("--- Test: float4 texture (4 channels) ---\n");
  const int N = 64;
  std::vector<float> host_data(N * 4);
  for (int i = 0; i < N * 4; i++) host_data[i] = (float)i * 0.01f;

  CuTexImage img;
  img.InitTexture(N, 1, 4);
  img.CopyFromHost(host_data.data());

  cudaTextureObject_t tex = img.CreateTextureObject();

  float4* d_out;
  cudaMalloc(&d_out, N * sizeof(float4));
  read_float4_kernel<<<(N+63)/64, 64>>>(d_out, N, tex);
  cudaDeviceSynchronize();

  std::vector<float4> result(N);
  cudaMemcpy(result.data(), d_out, N * sizeof(float4), cudaMemcpyDeviceToHost);

  int errors = 0;
  for (int i = 0; i < N; i++) {
    float* expected = &host_data[i * 4];
    float got[4] = {result[i].x, result[i].y, result[i].z, result[i].w};
    for (int c = 0; c < 4; c++) {
      if (fabsf(got[c] - expected[c]) > 1e-6f) {
        if (errors < 5)
          fprintf(stderr, "  [%d].%d expected %.6f, got %.6f\n", i, c, expected[c], got[c]);
        errors++;
      }
    }
  }
  CHECK(errors == 0, "float4 texture readback");
  printf("  float4: %d/%d components correct\n", N * 4 - errors, N * 4);

  img.DestroyTextureObject(tex);
  cudaFree(d_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: uint4 texture (4 channels, unsigned) — used by descriptor matching
// ─────────────────────────────────────────────────────────────────────────────
static void test_uint4_texture() {
  printf("--- Test: uint4 texture (4 channels, unsigned) ---\n");
  const int N = 64;
  // Simulate packed uint8 descriptors: 16 bytes per uint4 element
  std::vector<unsigned int> host_data(N * 4);
  for (int i = 0; i < N * 4; i++) {
    // Pack 4 bytes: [i*4, i*4+1, i*4+2, i*4+3] mod 256
    unsigned char b0 = (unsigned char)((i * 4 + 0) & 0xFF);
    unsigned char b1 = (unsigned char)((i * 4 + 1) & 0xFF);
    unsigned char b2 = (unsigned char)((i * 4 + 2) & 0xFF);
    unsigned char b3 = (unsigned char)((i * 4 + 3) & 0xFF);
    host_data[i] = (unsigned int)b0 | ((unsigned int)b1 << 8) |
                   ((unsigned int)b2 << 16) | ((unsigned int)b3 << 24);
  }

  CuTexImage img;
  img.InitTexture(N, 1, 4);
  // CopyFromHost copies _numChannel * N * sizeof(float) = 4*N*4 bytes — matches our uint data size
  img.CopyFromHost(host_data.data());

  // Use CreateTextureObjectUint — this is what MultiplyDescriptor should use
  cudaTextureObject_t tex = img.CreateTextureObjectUint();

  uint4* d_out;
  cudaMalloc(&d_out, N * sizeof(uint4));
  read_uint4_kernel<<<(N+63)/64, 64>>>(d_out, N, tex);
  cudaDeviceSynchronize();

  std::vector<uint4> result(N);
  cudaMemcpy(result.data(), d_out, N * sizeof(uint4), cudaMemcpyDeviceToHost);

  int errors = 0;
  for (int i = 0; i < N; i++) {
    unsigned int expected[4] = {host_data[i*4], host_data[i*4+1], host_data[i*4+2], host_data[i*4+3]};
    unsigned int got[4] = {result[i].x, result[i].y, result[i].z, result[i].w};
    for (int c = 0; c < 4; c++) {
      if (got[c] != expected[c]) {
        if (errors < 5)
          fprintf(stderr, "  [%d].%d expected 0x%08X, got 0x%08X\n", i, c, expected[c], got[c]);
        errors++;
      }
    }
  }
  CHECK(errors == 0, "uint4 texture readback (CreateTextureObjectUint)");
  printf("  uint4: %d/%d components correct\n", N * 4 - errors, N * 4);

  // Also test that CreateTextureObject (float) gives WRONG results for uint4 data
  // This validates that the fix is necessary
  cudaTextureObject_t tex_wrong = img.CreateTextureObject();  // float channel — wrong for uint data
  read_uint4_kernel<<<(N+63)/64, 64>>>(d_out, N, tex_wrong);
  cudaDeviceSynchronize();
  cudaMemcpy(result.data(), d_out, N * sizeof(uint4), cudaMemcpyDeviceToHost);

  int wrong_count = 0;
  for (int i = 0; i < N; i++) {
    unsigned int expected[4] = {host_data[i*4], host_data[i*4+1], host_data[i*4+2], host_data[i*4+3]};
    unsigned int got[4] = {result[i].x, result[i].y, result[i].z, result[i].w};
    for (int c = 0; c < 4; c++) {
      if (got[c] != expected[c]) wrong_count++;
    }
  }
  printf("  uint4 via float channel (wrong): %d/%d mismatches (expected >0 if fix is needed)\n",
         wrong_count, N * 4);

  img.DestroyTextureObject(tex);
  img.DestroyTextureObject(tex_wrong);
  cudaFree(d_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: int4 texture — used by histogram and list generation
// ─────────────────────────────────────────────────────────────────────────────
static void test_int4_texture() {
  printf("--- Test: int4 texture (4 channels, signed int) ---\n");
  const int N = 32;
  std::vector<int> host_data(N * 4);
  for (int i = 0; i < N * 4; i++) host_data[i] = i * 7 - 100;  // mix of positive and negative

  CuTexImage img;
  img.InitTexture(N, 1, 4);
  img.CopyFromHost(host_data.data());

  cudaTextureObject_t tex = img.CreateTextureObjectUint();

  int4* d_out;
  cudaMalloc(&d_out, N * sizeof(int4));
  read_int4_kernel<<<(N+31)/32, 32>>>(d_out, N, tex);
  cudaDeviceSynchronize();

  std::vector<int4> result(N);
  cudaMemcpy(result.data(), d_out, N * sizeof(int4), cudaMemcpyDeviceToHost);

  int errors = 0;
  for (int i = 0; i < N; i++) {
    int expected[4] = {host_data[i*4], host_data[i*4+1], host_data[i*4+2], host_data[i*4+3]};
    int got[4] = {result[i].x, result[i].y, result[i].z, result[i].w};
    for (int c = 0; c < 4; c++) {
      if (got[c] != expected[c]) {
        if (errors < 5)
          fprintf(stderr, "  [%d].%d expected %d, got %d\n", i, c, expected[c], got[c]);
        errors++;
      }
    }
  }
  CHECK(errors == 0, "int4 texture readback (CreateTextureObjectUint)");
  printf("  int4: %d/%d components correct\n", N * 4 - errors, N * 4);

  img.DestroyTextureObject(tex);
  cudaFree(d_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main() {
  printf("=== SiftGPU Texture Object Unit Test (CUDA 12.x migration) ===\n\n");

  // Check CUDA device
  int device_count = 0;
  cudaGetDeviceCount(&device_count);
  if (device_count == 0) {
    fprintf(stderr, "No CUDA devices found\n");
    return 1;
  }
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, 0);
  printf("Device: %s (SM %d.%d)\n\n", prop.name, prop.major, prop.minor);

  test_float_texture();
  test_float4_texture();
  test_uint4_texture();
  test_int4_texture();

  printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
  return g_fail > 0 ? 1 : 0;
}

#else
int main() {
  printf("CUDA not enabled, skipping texture object tests\n");
  return 0;
}
#endif
