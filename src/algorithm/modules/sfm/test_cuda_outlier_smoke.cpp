#include "cuda/cuda_sfm_state.h"

#include <cstdio>
#include <cstdlib>

using namespace insight;
using namespace insight::cuda;

int main()
{
    const int max_obs = 4096;
    std::printf("test_cuda_outlier_smoke — CudaSfMState incl. outlier scratch (n_obs=%d)\n", max_obs);
    CudaSfMState* s = gpu_sfm_state_create(1, 1, max_obs, 1);
    if (s == nullptr) {
        std::fprintf(stderr, "gpu_sfm_state_create failed\n");
        return 1;
    }
    if (s->d_reproj_reject_mark == nullptr || s->d_angle_csr_obs == nullptr) {
        std::fprintf(stderr, "outlier scratch buffers not allocated\n");
        gpu_sfm_state_free(s);
        return 1;
    }
    std::printf("  scratch OK (d_reproj_reject_mark, d_angle_*)\n");
    gpu_sfm_state_free(s);
    std::printf("OK\n");
    return 0;
}
