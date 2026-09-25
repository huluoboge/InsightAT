# InsightAT CLI tools (isat_*)
# Added after algorithm/ so geo_ransac, cuda_geo_ransac, sfm_module exist.

cmake_minimum_required(VERSION 3.16)

# isat_extract may set LINKER_LANGUAGE CUDA when PopSift is linked; ensure
# CUDA toolchain variables (e.g. CMAKE_CUDA_LINK_EXECUTABLE) are initialized.
if(CUDAToolkit_FOUND)
  enable_language(CUDA)
endif()

# ── Seed-eval common logic unit test ─────────────────────────────────────
add_executable(test_seed_eval_common test_seed_eval_common.cpp)
target_link_libraries(test_seed_eval_common
    PRIVATE
        glog::glog
)
target_include_directories(test_seed_eval_common
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_seed_eval_common PROPERTY FOLDER InsightAT/Tests)

# ─────────────────────────────────────────────────────────────
# CLI Tools (shared logging helper)
# ─────────────────────────────────────────────────────────────
add_library(insightat_tools_logging STATIC cli_logging.cpp)
target_include_directories(insightat_tools_logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(insightat_tools_logging PUBLIC glog::glog)
set_property(TARGET insightat_tools_logging PROPERTY FOLDER InsightAT/CLI)

# ─────────────────────────────────────────────────────────────
# CLI executables
# ─────────────────────────────────────────────────────────────

# isat_extract - Feature extraction tool
add_executable(isat_extract isat_extract.cpp)

target_link_libraries(isat_extract
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
)

if(TARGET sift_gpu)
    target_link_libraries(isat_extract
        PRIVATE
            sift_gpu
            ${GLEW_LIBRARIES}
            ${OPENGL_LIBRARIES}
    )
endif()

# In fully-static PopSift builds, rely on direct linkage from final executable
# so CUDA device-link can see PopSift CUDA objects explicitly.
if(TARGET PopSift::popsift)
    if(BUILD_SHARED_LIBS)
        target_link_libraries(isat_extract PRIVATE PopSift::popsift)
    else()
        # Static CUDA archives can drop device-link registration objects unless
        # the archive is force-loaded.
        target_link_libraries(isat_extract PRIVATE
            -Wl,--whole-archive
            PopSift::popsift
            -Wl,--no-whole-archive)
    endif()
endif()

# PopSift/CUDA static-device runtime may require explicit linkage at final executable link step.
if(TARGET CUDA::cudadevrt)
    target_link_libraries(isat_extract PRIVATE CUDA::cudadevrt)
elseif(DEFINED CUDA_CUDADEVRT_LIBRARY AND NOT "${CUDA_CUDADEVRT_LIBRARY}" STREQUAL "")
    target_link_libraries(isat_extract PRIVATE ${CUDA_CUDADEVRT_LIBRARY})
endif()
if(TARGET CUDA::cudart)
    target_link_libraries(isat_extract PRIVATE CUDA::cudart)
endif()

if(TARGET PopSift::popsift)
    # Force CUDA linker for final executable to resolve fatbin/device-runtime symbols
    # propagated from static PopSift CUDA objects.
    set_target_properties(isat_extract PROPERTIES LINKER_LANGUAGE CUDA)
    set_target_properties(isat_extract PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
endif()

target_include_directories(isat_extract
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_extract PROPERTY FOLDER InsightAT/CLI)

# isat_match - Feature matching tool
add_executable(isat_match isat_match.cpp)

target_link_libraries(isat_match
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
)

if(TARGET sift_gpu)
    target_link_libraries(isat_match
        PRIVATE
            sift_gpu
            ${GLEW_LIBRARIES}
            ${OPENGL_LIBRARIES}
    )
endif()

target_include_directories(isat_match
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_match PROPERTY FOLDER InsightAT/CLI)

# isat_cpu_cascade_hashing_match - CPU cascade hashing feature matching tool
add_executable(isat_cpu_cascade_hashing_match isat_cpu_cascade_hashing_match.cpp)

target_link_libraries(isat_cpu_cascade_hashing_match
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
)

target_include_directories(isat_cpu_cascade_hashing_match
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_cpu_cascade_hashing_match PROPERTY FOLDER InsightAT/CLI)

# isat_gpu_cascade_hashing_match - GPU cascade hashing feature matching tool (CUDA only)
if(CUDAToolkit_FOUND)
    add_executable(isat_gpu_cascade_hashing_match isat_gpu_cascade_hashing_match.cpp)

    target_link_libraries(isat_gpu_cascade_hashing_match
        PRIVATE
            insightat_tools_logging
            InsightATAlgorithm
            glog::glog
            cudart
    )

    target_include_directories(isat_gpu_cascade_hashing_match
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
            ${CMAKE_SOURCE_DIR}/third_party
    )

    set_property(TARGET isat_gpu_cascade_hashing_match PROPERTY FOLDER InsightAT/CLI)
    message(STATUS "CLI: isat_gpu_cascade_hashing_match (CUDA cascade hashing) enabled")
else()
    message(STATUS "CLI: isat_gpu_cascade_hashing_match (CUDA cascade hashing) disabled - CUDAToolkit not found")
endif()

# isat_retrieve - Image pair retrieval tool
add_executable(isat_retrieve isat_retrieve.cpp)

target_link_libraries(isat_retrieve
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        stlplus3
)

target_include_directories(isat_retrieve
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_retrieve PROPERTY FOLDER InsightAT/CLI)

# isat_train_vlad - VLAD codebook training tool
add_executable(isat_train_vlad isat_train_vlad.cpp)

target_link_libraries(isat_train_vlad
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
)

target_include_directories(isat_train_vlad
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_train_vlad PROPERTY FOLDER InsightAT/CLI)

# isat_geo - Two-view geometry estimation (F / E / H) using GPU RANSAC
add_executable(isat_geo isat_geo.cpp)

target_link_libraries(isat_geo
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        geo_ransac
        glog::glog
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
)

target_include_directories(isat_geo
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
        ${CMAKE_SOURCE_DIR}/src/algorithm/modules/geometry
)

set_property(TARGET isat_geo PROPERTY FOLDER InsightAT/CLI)

# isat_project - Inspect/list/export project and ATTask data
add_executable(isat_project isat_project.cpp)

target_link_libraries(isat_project
    PRIVATE
        insightat_tools_logging
        InsightATDatabase
        glog::glog
)

target_include_directories(isat_project
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_project PROPERTY FOLDER InsightAT/CLI)

# isat_twoview - Per-pair two-view reconstruction (focal estimation + triangulation + LM-BA)
# add_executable(isat_twoview isat_twoview.cpp)
# ... temporarily excluded; re-enable after stepwise re-implementation

# isat_tracks - Build tracks from match + geo, write single IDC for SfM
add_executable(isat_tracks isat_tracks.cpp)

target_link_libraries(isat_tracks
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        sfm_module
        glog::glog
)

target_include_directories(isat_tracks
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_tracks PROPERTY FOLDER InsightAT/CLI)

# isat_calibrate - Global focal-length refinement across image pairs
# add_executable(isat_calibrate isat_calibrate.cpp)
# ... temporarily excluded

# isat_incremental_sfm - Incremental SfM pipeline (tracks IDC + project JSON + pairs + geo)
add_executable(isat_incremental_sfm isat_incremental_sfm.cpp)
target_link_libraries(isat_incremental_sfm
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        sfm_module
        glog::glog
)
target_include_directories(isat_incremental_sfm
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET isat_incremental_sfm PROPERTY FOLDER InsightAT/CLI)

# isat_seed_eval - Multi-strategy seed-pair evaluation CLI
add_executable(isat_seed_eval isat_seed_eval.cpp)
target_link_libraries(isat_seed_eval
    PRIVATE
        insightat_tools_logging
        glog::glog
)
target_include_directories(isat_seed_eval
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET isat_seed_eval PROPERTY FOLDER InsightAT/CLI)

# isat_camera_estimator - Per-group EXIF-based camera intrinsics estimator
add_executable(isat_camera_estimator isat_camera_estimator.cpp)

target_link_libraries(isat_camera_estimator
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        InsightATDatabase
        ImageIO
        glog::glog
        ${GDAL_LIBRARIES}
)

target_include_directories(isat_camera_estimator
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)

set_property(TARGET isat_camera_estimator PROPERTY FOLDER InsightAT/CLI)

# isat_sfm - End-to-end SfM pipeline driver (invokes sibling tools as subprocesses)
add_executable(isat_sfm isat_sfm.cpp)

# Inject compile-time configuration based on CUDA availability
if(CUDAToolkit_FOUND)
    target_compile_definitions(isat_sfm PRIVATE
        ISAT_SFM_DEFAULT_EXTRACT_BACKEND="cuda"
        ISAT_SFM_DEFAULT_MATCH_BACKEND="cuda"
        ISAT_SFM_DEFAULT_GEO_BACKEND="cuda"
        ISAT_SFM_DEFAULT_EXTRACT_IMPL="popsift"
        ISAT_SFM_DEFAULT_MATCH_IMPL="cascade-gpu"
    )
else()
    target_compile_definitions(isat_sfm PRIVATE
        ISAT_SFM_DEFAULT_EXTRACT_BACKEND="glsl"
        ISAT_SFM_DEFAULT_MATCH_BACKEND="cpu"
        ISAT_SFM_DEFAULT_GEO_BACKEND="gpu"
        ISAT_SFM_DEFAULT_EXTRACT_IMPL="siftgpu"
        ISAT_SFM_DEFAULT_MATCH_IMPL="cascade"
    )
endif()

target_link_libraries(isat_sfm
    PRIVATE
        insightat_tools_logging
        glog::glog
)
target_include_directories(isat_sfm
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET isat_sfm PROPERTY FOLDER InsightAT/CLI)

# isat_retrieval_match - Discover image pairs via exhaustive low-res matching + F verification
add_executable(isat_retrieval_match isat_retrieval_match.cpp)
target_link_libraries(isat_retrieval_match
    PRIVATE
        insightat_tools_logging
        glog::glog
)
target_include_directories(isat_retrieval_match
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET isat_retrieval_match PROPERTY FOLDER InsightAT/CLI)

# isat_undistort - Undistort registered images for 3DGS / COLMAP-compatible output
add_executable(isat_undistort isat_undistort.cpp)
target_link_libraries(isat_undistort
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
)
target_include_directories(isat_undistort
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET isat_undistort PROPERTY FOLDER InsightAT/CLI)

# CUDA geo extras (cuda_geo_ransac built in algorithm/modules/geometry)
if(TARGET cuda_geo_ransac)
  target_link_libraries(isat_geo PRIVATE cuda_geo_ransac CUDA::cudart)

  # isat_geo_cuda — pure CUDA pipeline: F+E+H RANSAC, E decompose, triangulate all on GPU
  add_executable(isat_geo_cuda isat_geo_cuda.cpp)
  target_link_libraries(isat_geo_cuda
      PRIVATE
          insightat_tools_logging
          InsightATAlgorithm
          cuda_geo_ransac
          sfm_module
          glog::glog
          CUDA::cudart
  )
  target_include_directories(isat_geo_cuda
      PRIVATE
          ${CMAKE_SOURCE_DIR}/src
          ${CMAKE_SOURCE_DIR}/third_party
          ${CMAKE_SOURCE_DIR}/src/algorithm/modules/geometry
  )
  target_compile_definitions(isat_geo_cuda PRIVATE INSIGHTAT_HAS_CUDA_GEO=1)
  set_property(TARGET isat_geo_cuda PROPERTY FOLDER InsightAT/CLI)
  message(STATUS "CLI: isat_geo_cuda enabled (CUDA E decompose + triangulate)")
endif()
