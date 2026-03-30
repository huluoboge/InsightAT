#pragma once
/**
 * @file  bundle_adjustment_pba.h
 * @brief PBA (Parallel Bundle Adjustment) wrapper for InsightAT's BAInput / BAResult.
 *
 * Camera model mapping
 * ─────────────────────
 * PBA does NOT have a principal point; the caller must subtract (cx, cy) from
 * observations before passing them in.  We do this internally here.
 *
 * PBA uses a single focal length f (assumes fx ≈ fy).  We set f = fx from the
 * BAInput intrinsics and call SetFixedIntrinsics(true), so only poses and 3-D
 * points are optimised.
 *
 * Radial distortion is disabled (PBA_NO_DISTORTION) for now.
 *
 * This is an experimental wrapper – the API intentionally mirrors
 * global_bundle_analytic() so the call-site can swap them with one line.
 */

#include "bundle_adjustment_analytic.h"

namespace insight {
namespace sfm {

/**
 * Run global BA using PBA (CPU, fixed intrinsics, no distortion).
 *
 * Intrinsics in @p input are NOT modified; only poses and 3-D points are
 * returned in @p result.  result->cameras is copied unchanged from input.
 *
 * @return true  if PBA reports at least 1 successful LM iteration.
 * @return false if PBA fails or the input is empty.
 */
bool global_bundle_pba(const BAInput& input, BAResult* result,
                       int max_iterations = 50);

}  // namespace sfm
}  // namespace insight
