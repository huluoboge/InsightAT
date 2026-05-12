#include "cuda/cuda_triangulation.cuh"

#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>

using namespace insight;
using namespace insight::cuda;

int main()
{
    const int max_obs = 1000, max_tracks = 100, max_pairs = 500;
    std::printf("test_cuda_triangulation_smoke — GpuTriContext create/free\n");
    GpuTriContext* ctx = gpu_tri_ctx_create(max_obs, max_tracks, max_pairs);
    if (ctx == nullptr) {
        std::fprintf(stderr, "gpu_tri_ctx_create returned null\n");
        return 1;
    }
    std::printf("  caps: max_obs=%d  max_tracks=%d  max_pairs=%d  context OK\n", max_obs, max_tracks, max_pairs);
    gpu_tri_ctx_free(ctx);
    std::printf("OK\n");
    return 0;
}
