#!/usr/bin/env bash
# Shared CUDA 12 + cuDSS bundling helpers for AppImage / .deb (Ubuntu 22.04 & 24.04).
# Sourced by packaging/deb/package.sh and packaging/appimage/build.sh.
#
# Avoids host update-alternatives pointing libcudss.so.0 at a CUDA 13 tree
# (which needs libcublas.so.13). Always prefer CUDSS_ROOT / libcudss/12.

# Resolve a CUDA-12-compatible directory that contains libcudss.so*.
# Optional overrides: CUDSS_LIB_DIR, CUDSS_ROOT.
insightat_resolve_cudss_lib_dir() {
  if [[ -n "${CUDSS_LIB_DIR:-}" && -d "${CUDSS_LIB_DIR}" ]]; then
    printf '%s\n' "${CUDSS_LIB_DIR}"
    return 0
  fi
  if [[ -n "${CUDSS_ROOT:-}" ]]; then
    for cand in "${CUDSS_ROOT}/lib" "${CUDSS_ROOT}/lib64"; do
      if [[ -d "${cand}" ]] && compgen -G "${cand}/libcudss.so*" >/dev/null; then
        printf '%s\n' "${cand}"
        return 0
      fi
    done
  fi
  for cand in \
    /usr/lib/x86_64-linux-gnu/libcudss/12 \
    /lib/x86_64-linux-gnu/libcudss/12 \
    /usr/local/cuda-12.8/lib64 \
    /usr/local/cuda/lib64; do
    if [[ -d "${cand}" ]] && compgen -G "${cand}/libcudss.so*" >/dev/null; then
      printf '%s\n' "${cand}"
      return 0
    fi
  done
  return 1
}

# Copy one shared library into DEST_DIR, preserving SONAME symlink name.
# Usage: insightat_copy_shared_lib DEST_DIR SRC_PATH
insightat_copy_shared_lib() {
  local dest_dir="$1"
  local src="$2"
  [[ -n "${src}" && -e "${src}" ]] || return 0
  mkdir -p "${dest_dir}"
  local real base link_name
  real="$(readlink -f "${src}")"
  base="$(basename "${real}")"
  if [[ ! -e "${dest_dir}/${base}" ]]; then
    # Avoid cp -a: NVIDIA redistributables may not allow preserving ownership.
    cp --preserve=mode,timestamps "${real}" "${dest_dir}/${base}"
  fi
  link_name="$(basename "${src}")"
  if [[ "${link_name}" != "${base}" && ! -e "${dest_dir}/${link_name}" ]]; then
    ln -sfn "${base}" "${dest_dir}/${link_name}"
  fi
}

# Bundle CUDA 12 runtime + matching cuDSS into DEST_DIR.
# Requires CUDA_LIBS_DIR. Uses CUDSS_LIB_DIR / CUDSS_ROOT / libcudss/12.
# Usage: insightat_bundle_cuda_libs DEST_DIR
insightat_bundle_cuda_libs() {
  local dest_dir="$1"
  local cuda_dir="${CUDA_LIBS_DIR:-/usr/local/cuda-12.8/lib64}"
  local cudss_dir pat f

  [[ -d "${cuda_dir}" ]] || {
    echo "ERROR: CUDA_LIBS_DIR does not exist: ${cuda_dir}" >&2
    return 1
  }
  mkdir -p "${dest_dir}"

  echo "Bundling CUDA libs from ${cuda_dir} -> ${dest_dir}"
  for pat in \
    libcudart.so* \
    libcublas.so* \
    libcublasLt.so* \
    libcusolver.so* \
    libcusparse.so* \
    libnvJitLink.so* \
    libcufft.so* \
    libnvrtc.so*; do
    for f in "${cuda_dir}"/${pat}; do
      [[ -e "${f}" ]] || continue
      insightat_copy_shared_lib "${dest_dir}" "${f}"
    done
  done

  cudss_dir="$(insightat_resolve_cudss_lib_dir)" || {
    echo "ERROR: could not locate CUDA-12-compatible libcudss (set CUDSS_LIB_DIR or CUDSS_ROOT)" >&2
    return 1
  }
  echo "Bundling cuDSS libs from ${cudss_dir} -> ${dest_dir}"
  # Force-replace any libcudss previously pulled in via ldd (may be CUDA 13).
  rm -f "${dest_dir}"/libcudss.so*
  for f in "${cudss_dir}"/libcudss.so*; do
    [[ -e "${f}" ]] || continue
    insightat_copy_shared_lib "${dest_dir}" "${f}"
  done
  if ! compgen -G "${dest_dir}/libcudss.so*" >/dev/null; then
    echo "ERROR: failed to bundle libcudss into ${dest_dir}" >&2
    return 1
  fi
}

# Verify ELF binaries under BIN_DIR resolve CUDA/cuDSS via LIB_DIR and do not
# depend on libcublas.so.13 (CUDA 13 cuDSS leak).
# Usage: insightat_verify_cuda_linkage BIN_DIR LIB_DIR
insightat_verify_cuda_linkage() {
  local bin_dir="$1"
  local lib_dir="$2"
  local binary line missing=0 bad13=0 checked=0

  for binary in "${bin_dir}"/*; do
    [[ -f "${binary}" && -x "${binary}" && ! -L "${binary}" ]] || continue
    # Skip non-ELF (wrapper scripts).
    if ! file -b "${binary}" 2>/dev/null | grep -qE '^ELF'; then
      continue
    fi
    checked=$((checked + 1))

    # Do not inherit caller LD_LIBRARY_PATH — it can hide a CUDA-13 cuDSS leak.
    while IFS= read -r line; do
      case "${line}" in
        *libcublas.so.13*)
          echo "ERROR: $(basename "${binary}") still references libcublas.so.13 (CUDA 13 cuDSS leak): ${line}" >&2
          bad13=1
          ;;
        *libcublas.so.*\ =\>\ not\ found*|*libcudss.so.*\ =\>\ not\ found*|*libcudart.so.*\ =\>\ not\ found*|*libcusolver.so.*\ =\>\ not\ found*|*libcusparse.so.*\ =\>\ not\ found*)
          echo "ERROR: unresolved CUDA dependency in $(basename "${binary}"): ${line}" >&2
          missing=1
          ;;
      esac
    done < <(env LD_LIBRARY_PATH="${lib_dir}" ldd "${binary}" 2>/dev/null || true)
  done

  if [[ "${checked}" -eq 0 ]]; then
    echo "ERROR: no ELF binaries found under ${bin_dir} to verify" >&2
    return 1
  fi
  if [[ "${bad13}" -ne 0 ]]; then
    echo "ERROR: packaged binaries depend on libcublas.so.13; bundle CUDA-12 cuDSS instead" >&2
    return 1
  fi
  if [[ "${missing}" -ne 0 ]]; then
    echo "ERROR: packaged binaries are missing CUDA/cuDSS libraries under ${lib_dir}" >&2
    return 1
  fi
  echo "CUDA/cuDSS linkage check OK (${checked} binaries, ${bin_dir} + ${lib_dir})"
}
