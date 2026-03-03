/**
 * gpu_twoview_sfm.cpp
 * GPU two-view SfM primitives – EGL + OpenGL 4.3 Compute Shaders.
 *
 * Two programs:
 *   1. Triangulate  – per-thread: linear DLT for one 2-D correspondence.
 *   2. Residuals    – per-thread: Huber-weighted reprojection error for one
 *                     3-D point, both cameras.
 *
 * SSBO layout:
 *   triangulate:
 *     binding 0  readonly  float pts_n[4*n]  = [x1n,y1n,x2n,y2n, ...]
 *     binding 1  readonly  float cam[12]     = R[9] row-major, t[3]
 *     binding 2  writeonly float X_out[4*n]  = [X,Y,Z,valid, ...] per point
 *
 *   residuals:
 *     binding 0  readonly  float pts_px[4*n] = [u1,v1,u2,v2, ...]
 *     binding 1  readonly  float X[3*n]      = [X,Y,Z, ...]
 *     binding 2  readonly  float cam[12]     = R[9], t[3]
 *     binding 3  writeonly float out[5*n]    = [r0,r1,r2,r3,valid, ...]
 *     binding 4  readonly  float intrinsics[4] = [f, cx, cy, huber_k]
 */

#include "gpu_twoview_sfm.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glew.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────────────

static int s_local_size = 64;

static EGLDisplay s_egl_dpy = EGL_NO_DISPLAY;
static EGLContext s_egl_ctx = EGL_NO_CONTEXT;
static EGLSurface s_egl_surf = EGL_NO_SURFACE;

static GLuint s_prog_tri = 0; // triangulation shader
static GLuint s_prog_res = 0; // residuals shader

// SSBOs, regrown lazily
static GLuint s_ssbo_pts_in = 0;  // float[4*n]  pts input (tri or residuals)
static GLuint s_ssbo_cam = 0;     // float[12]   R[9] + t[3]
static GLuint s_ssbo_X = 0;       // float[3*n]  3D points (residuals input)
static GLuint s_ssbo_out_tri = 0; // float[4*n]  tri output (X,Y,Z,valid)
static GLuint s_ssbo_out_res = 0; // float[5*n]  residuals output
static GLuint s_ssbo_intr = 0;    // float[4]    f,cx,cy,huber_k

static int s_cap_n = 0; // current SSBO capacity in points

// ─────────────────────────────────────────────────────────────────────────────
// Triangulation compute shader – one thread per correspondence
// ─────────────────────────────────────────────────────────────────────────────

static const char* TRI_SHADER_TPL = R"GLSL(
#version 430
#define LOCAL_SIZE %%LOCAL_SIZE%%
layout(local_size_x = LOCAL_SIZE) in;

// pts_n: [x1n, y1n, x2n, y2n] per point
layout(std430, binding=0) readonly  buffer PtsBuf { float pts_n[]; };
// cam: R[9] row-major, t[3]
layout(std430, binding=1) readonly  buffer CamBuf { float cam[];   };
// out: [X, Y, Z, valid(1.0 or 0.0)] per point
layout(std430, binding=2) writeonly buffer OutBuf { float X_out[]; };

uniform int u_n;  // number of points

const float EPS = 1e-9;
const float NAN_VAL = uintBitsToFloat(0x7FC00000u); // quiet NaN

// ─────────────────────────────────────────────────────────────────────────────
// 4×4 symmetric Cholesky Inverse Power Iteration
// Finds null vector of a 4×4 system A (given as 4 rows of 4 floats).
// Returns smallest-eigenvalue eigenvector of AᵀA.
// ─────────────────────────────────────────────────────────────────────────────
void null_vector4(in float A[16], out float x[4]) {
    // Build B = AᵀA (4×4 symmetric)
    float B[16];
    for (int i = 0; i < 16; i++) B[i] = 0.0;
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                B[i*4+j] += A[k*4+i] * A[k*4+j];

    // Regularise: B += mu*I
    float tr = B[0] + B[5] + B[10] + B[15];
    float mu = tr * 1e-3 + 1e-30;
    B[0]  += mu;  B[5]  += mu;  B[10] += mu;  B[15] += mu;

    // Cholesky: L·Lᵀ = B  (stored in lower triangle of B)
    for (int j = 0; j < 4; j++) {
        float s = B[j*4+j];
        for (int k = 0; k < j; k++) s -= B[j*4+k]*B[j*4+k];
        B[j*4+j] = sqrt(max(s, 1e-30));
        for (int i = j+1; i < 4; i++) {
            s = B[i*4+j];
            for (int k = 0; k < j; k++) s -= B[i*4+k]*B[j*4+k];
            B[i*4+j] = s / B[j*4+j];
        }
    }

    // Inverse power iteration: 8 rounds of (L·Lᵀ)\x
    float y[4];
    x[0] = 0.5; x[1] = 0.5; x[2] = 0.5; x[3] = 0.5;
    for (int iter = 0; iter < 8; iter++) {
        // Forward sub: solve L·y = x
        for (int i = 0; i < 4; i++) {
            y[i] = x[i];
            for (int j = 0; j < i; j++) y[i] -= B[i*4+j]*y[j];
            y[i] /= B[i*4+i];
        }
        // Back sub: solve Lᵀ·x = y  (Lᵀ[i][j] = L[j][i] = B[j*4+i])
        for (int i = 3; i >= 0; i--) {
            x[i] = y[i];
            for (int j = i+1; j < 4; j++) x[i] -= B[j*4+i]*x[j];
            x[i] /= B[i*4+i];
        }
        // Normalise
        float nrm = 0.0;
        for (int i = 0; i < 4; i++) nrm += x[i]*x[i];
        nrm = sqrt(nrm) + 1e-30;
        for (int i = 0; i < 4; i++) x[i] /= nrm;
    }
}

void main() {
    int idx = int(gl_GlobalInvocationID.x);
    if (idx >= u_n) return;

    float x1n = pts_n[idx*4+0];
    float y1n = pts_n[idx*4+1];
    float x2n = pts_n[idx*4+2];
    float y2n = pts_n[idx*4+3];

    // R (row-major) and t
    float R00=cam[0], R01=cam[1], R02=cam[2];
    float R10=cam[3], R11=cam[4], R12=cam[5];
    float R20=cam[6], R21=cam[7], R22=cam[8];
    float tx=cam[9],  ty=cam[10], tz=cam[11];

    // Build 4×4 DLT matrix  A·X_h = 0 ,  X_h = (X,Y,Z,W)
    // cam1 = [I|0]:
    //   row 0:  (-1, 0,  x1n, 0)
    //   row 1:  ( 0,-1,  y1n, 0)
    // cam2 = [R|t]:
    //   row 2:  (x2n*R20-R00, x2n*R21-R01, x2n*R22-R02, x2n*tz-tx)
    //   row 3:  (y2n*R20-R10, y2n*R21-R11, y2n*R22-R12, y2n*tz-ty)
    float A[16];
    A[0]  = -1.0;           A[1]  = 0.0;           A[2]  = x1n;                A[3]  = 0.0;
    A[4]  = 0.0;            A[5]  = -1.0;          A[6]  = y1n;                A[7]  = 0.0;
    A[8]  = x2n*R20 - R00;  A[9]  = x2n*R21 - R01; A[10] = x2n*R22 - R02;     A[11] = x2n*tz - tx;
    A[12] = y2n*R20 - R10;  A[13] = y2n*R21 - R11; A[14] = y2n*R22 - R12;     A[15] = y2n*tz - ty;

    float Xh[4];
    null_vector4(A, Xh);

    // Dehomogenize
    float W = Xh[3];
    if (abs(W) < EPS) {
        X_out[idx*4+0] = NAN_VAL;
        X_out[idx*4+1] = NAN_VAL;
        X_out[idx*4+2] = NAN_VAL;
        X_out[idx*4+3] = 0.0;
        return;
    }
    float X = Xh[0] / W;
    float Y = Xh[1] / W;
    float Z = Xh[2] / W;

    // Check depth in cam1 and cam2
    float depth1 = Z;
    float depth2 = R20*X + R21*Y + R22*Z + tz;

    if (depth1 <= EPS || depth2 <= EPS) {
        X_out[idx*4+0] = NAN_VAL;
        X_out[idx*4+1] = NAN_VAL;
        X_out[idx*4+2] = NAN_VAL;
        X_out[idx*4+3] = 0.0;
        return;
    }

    X_out[idx*4+0] = X;
    X_out[idx*4+1] = Y;
    X_out[idx*4+2] = Z;
    X_out[idx*4+3] = 1.0;  // valid
}
)GLSL";

// ─────────────────────────────────────────────────────────────────────────────
// Residuals compute shader – one thread per 3-D point
// ─────────────────────────────────────────────────────────────────────────────

static const char* RES_SHADER_TPL = R"GLSL(
#version 430
#define LOCAL_SIZE %%LOCAL_SIZE%%
layout(local_size_x = LOCAL_SIZE) in;

// pixel observations: [u1,v1,u2,v2] per point
layout(std430, binding=0) readonly  buffer PtsPxBuf  { float pts_px[]; };
// 3D points: [X,Y,Z] per point
layout(std430, binding=1) readonly  buffer XBuf      { float X_pts[]; };
// cam: R[9] row-major, t[3]
layout(std430, binding=2) readonly  buffer CamBuf    { float cam[];    };
// out: [r0,r1,r2,r3, valid] per point
layout(std430, binding=3) writeonly buffer OutBuf    { float out_res[]; };
// intrinsics: [f, cx, cy, huber_k]
layout(std430, binding=4) readonly  buffer IntrBuf   { float intr[];   };

uniform int u_n;

void main() {
    int idx = int(gl_GlobalInvocationID.x);
    if (idx >= u_n) return;

    float X  = X_pts[idx*3+0];
    float Y  = X_pts[idx*3+1];
    float Z  = X_pts[idx*3+2];

    // NaN check: if point is invalid, write zeros
    if (isnan(X) || isnan(Y) || isnan(Z)) {
        out_res[idx*5+0] = 0.0;
        out_res[idx*5+1] = 0.0;
        out_res[idx*5+2] = 0.0;
        out_res[idx*5+3] = 0.0;
        out_res[idx*5+4] = 0.0;  // invalid
        return;
    }

    float f  = intr[0];
    float cx = intr[1];
    float cy = intr[2];
    float hk = intr[3];

    float R00=cam[0], R01=cam[1], R02=cam[2];
    float R10=cam[3], R11=cam[4], R12=cam[5];
    float R20=cam[6], R21=cam[7], R22=cam[8];
    float tx=cam[9],  ty=cam[10], tz=cam[11];

    float obs_u1 = pts_px[idx*4+0];
    float obs_v1 = pts_px[idx*4+1];
    float obs_u2 = pts_px[idx*4+2];
    float obs_v2 = pts_px[idx*4+3];

    // Camera 1 (identity): depth = Z
    float depth1 = Z;
    if (depth1 <= 1e-9) {
        out_res[idx*5+0] = 0.0;  out_res[idx*5+1] = 0.0;
        out_res[idx*5+2] = 0.0;  out_res[idx*5+3] = 0.0;
        out_res[idx*5+4] = 0.0;
        return;
    }
    float pred_u1 = f * X / depth1 + cx;
    float pred_v1 = f * Y / depth1 + cy;

    // Camera 2
    float Xc2 = R00*X + R01*Y + R02*Z + tx;
    float Yc2 = R10*X + R11*Y + R12*Z + ty;
    float Zc2 = R20*X + R21*Y + R22*Z + tz;
    if (abs(Zc2) <= 1e-9 || Zc2 <= 1e-9) {
        out_res[idx*5+0] = 0.0;  out_res[idx*5+1] = 0.0;
        out_res[idx*5+2] = 0.0;  out_res[idx*5+3] = 0.0;
        out_res[idx*5+4] = 0.0;
        return;
    }
    float pred_u2 = f * Xc2 / Zc2 + cx;
    float pred_v2 = f * Yc2 / Zc2 + cy;

    // Raw residuals
    float r0 = pred_u1 - obs_u1;
    float r1 = pred_v1 - obs_v1;
    float r2 = pred_u2 - obs_u2;
    float r3 = pred_v2 - obs_v2;

    // Huber weighting per residual
    float w0 = (abs(r0) <= hk) ? 1.0 : hk / (abs(r0) + 1e-30);
    float w1 = (abs(r1) <= hk) ? 1.0 : hk / (abs(r1) + 1e-30);
    float w2 = (abs(r2) <= hk) ? 1.0 : hk / (abs(r2) + 1e-30);
    float w3 = (abs(r3) <= hk) ? 1.0 : hk / (abs(r3) + 1e-30);

    out_res[idx*5+0] = w0 * r0;
    out_res[idx*5+1] = w1 * r1;
    out_res[idx*5+2] = w2 * r2;
    out_res[idx*5+3] = w3 * r3;
    out_res[idx*5+4] = 1.0;  // valid
}
)GLSL";

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::string injectDefine(const char* tpl, int local_size) {
  std::string src(tpl);
  const char* placeholder = "%%LOCAL_SIZE%%";
  std::string val = std::to_string(local_size);
  size_t pos;
  while ((pos = src.find(placeholder)) != std::string::npos)
    src.replace(pos, strlen(placeholder), val);
  return src;
}

static GLuint compileProg(const std::string& src) {
  const char* csrc = src.c_str();
  GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(shader, 1, &csrc, nullptr);
  glCompileShader(shader);
  GLint ok;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[4096];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    fprintf(stderr, "[gpu_twoview_sfm] Shader compile error:\n%s\n", log);
    glDeleteShader(shader);
    return 0;
  }
  GLuint prog = glCreateProgram();
  glAttachShader(prog, shader);
  glLinkProgram(prog);
  glGetProgramiv(prog, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[4096];
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    fprintf(stderr, "[gpu_twoview_sfm] Program link error:\n%s\n", log);
    glDeleteShader(shader);
    glDeleteProgram(prog);
    return 0;
  }
  glDeleteShader(shader);
  return prog;
}

static GLuint makeSSBO(GLsizeiptr bytes) {
  GLuint buf;
  glGenBuffers(1, &buf);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
  glBufferData(GL_SHADER_STORAGE_BUFFER, bytes, nullptr, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  return buf;
}

static void resizeSSBOs(int n) {
  if (n <= s_cap_n)
    return; // already large enough
  int newcap = std::max(n, 1024);

  auto resize1 = [](GLuint& buf, GLsizeiptr bytes) {
    if (buf)
      glDeleteBuffers(1, &buf);
    buf = makeSSBO(bytes);
  };

  resize1(s_ssbo_pts_in, (GLsizeiptr)(4 * newcap * sizeof(float)));
  resize1(s_ssbo_X, (GLsizeiptr)(3 * newcap * sizeof(float)));
  resize1(s_ssbo_out_tri, (GLsizeiptr)(4 * newcap * sizeof(float)));
  resize1(s_ssbo_out_res, (GLsizeiptr)(5 * newcap * sizeof(float)));

  s_cap_n = newcap;
}

static void uploadBuf(GLuint buf, const void* data, GLsizeiptr bytes) {
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
  glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, bytes, data);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

static void downloadBuf(GLuint buf, void* data, GLsizeiptr bytes) {
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
  glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, bytes, data);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// EGL initialisation (mirrors gpu_geo_ransac: surfaceless first, pbuffer fallback)
// ─────────────────────────────────────────────────────────────────────────────

int gpu_twoview_init(void) {
  /* ── EGL ──────────────────────────────────────────────────────────────── */
  s_egl_dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (s_egl_dpy == EGL_NO_DISPLAY) {
    fprintf(stderr, "[gpu_twoview_sfm] eglGetDisplay failed\n");
    return -1;
  }
  EGLint major, minor;
  if (!eglInitialize(s_egl_dpy, &major, &minor)) {
    fprintf(stderr, "[gpu_twoview_sfm] eglInitialize failed\n");
    return -1;
  }

  /* Bind OpenGL (not OpenGL ES) */
  if (!eglBindAPI(EGL_OPENGL_API)) {
    fprintf(stderr, "[gpu_twoview_sfm] eglBindAPI(EGL_OPENGL_API) failed\n");
    return -1;
  }

  /* Try surfaceless context (EGL_KHR_surfaceless_context) */
  bool surfaceless = false;
  {
    const char* exts = eglQueryString(s_egl_dpy, EGL_EXTENSIONS);
    if (exts && strstr(exts, "EGL_KHR_surfaceless_context"))
      surfaceless = true;
  }

  static const EGLint cfg_attr[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig cfg;
  EGLint ncfg = 0;
  if (!eglChooseConfig(s_egl_dpy, cfg_attr, &cfg, 1, &ncfg) || ncfg < 1) {
    fprintf(stderr, "[gpu_twoview_sfm] No suitable EGL config found\n");
    return -1;
  }

  static const EGLint ctx_attr[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 3,
                                    EGL_NONE};
  s_egl_ctx = eglCreateContext(s_egl_dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
  if (s_egl_ctx == EGL_NO_CONTEXT) {
    fprintf(stderr, "[gpu_twoview_sfm] eglCreateContext failed (need OpenGL 4.3)\n");
    return -1;
  }

  if (surfaceless) {
    s_egl_surf = EGL_NO_SURFACE;
    if (!eglMakeCurrent(s_egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, s_egl_ctx)) {
      surfaceless = false;
    }
  }
  if (!surfaceless) {
    static const EGLint pb_attr[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    s_egl_surf = eglCreatePbufferSurface(s_egl_dpy, cfg, pb_attr);
    if (s_egl_surf == EGL_NO_SURFACE ||
        !eglMakeCurrent(s_egl_dpy, s_egl_surf, s_egl_surf, s_egl_ctx)) {
      fprintf(stderr, "[gpu_twoview_sfm] eglMakeCurrent failed\n");
      return -1;
    }
  }

  /* ── GLEW ─────────────────────────────────────────────────────────────── */
  // glewExperimental allows loading extensions without a GLX/WGL display.
  // glewInit() may return non-GLEW_OK on surfaceless EGL because it tries to
  // initialise GLX extensions that don't exist; this is harmless as long as
  // the core GL 4.3 functions (glDispatchCompute, SSBOs) are available.
  glewExperimental = GL_TRUE;
  GLenum glew_err = glewInit();
  // Clear any GL error left by glewInit (common with EGL surfaceless)
  while (glGetError() != GL_NO_ERROR) {
  }

  if (glew_err != GLEW_OK) {
    fprintf(stderr, "[gpu_twoview_sfm] glewInit note: %s (checking GL 4.3 manually)\n",
            glewGetErrorString(glew_err));
  }

  // Verify that compute shader support was actually loaded
  if (!glDispatchCompute) {
    fprintf(stderr, "[gpu_twoview_sfm] glDispatchCompute not found – need OpenGL 4.3+\n");
    return -1;
  }

  /* ── Compile shaders ──────────────────────────────────────────────────── */
  s_prog_tri = compileProg(injectDefine(TRI_SHADER_TPL, s_local_size));
  if (!s_prog_tri)
    return -1;

  s_prog_res = compileProg(injectDefine(RES_SHADER_TPL, s_local_size));
  if (!s_prog_res) {
    glDeleteProgram(s_prog_tri);
    s_prog_tri = 0;
    return -1;
  }

  /* ── Permanent small SSBOs ────────────────────────────────────────────── */
  s_ssbo_cam = makeSSBO(12 * sizeof(float));
  s_ssbo_intr = makeSSBO(4 * sizeof(float));

  return 0;
}

void gpu_twoview_shutdown(void) {
  auto del = [](GLuint& b) {
    if (b) {
      glDeleteBuffers(1, &b);
      b = 0;
    }
  };
  del(s_ssbo_pts_in);
  del(s_ssbo_cam);
  del(s_ssbo_X);
  del(s_ssbo_out_tri);
  del(s_ssbo_out_res);
  del(s_ssbo_intr);

  if (s_prog_tri) {
    glDeleteProgram(s_prog_tri);
    s_prog_tri = 0;
  }
  if (s_prog_res) {
    glDeleteProgram(s_prog_res);
    s_prog_res = 0;
  }

  if (s_egl_dpy != EGL_NO_DISPLAY) {
    eglMakeCurrent(s_egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (s_egl_surf != EGL_NO_SURFACE)
      eglDestroySurface(s_egl_dpy, s_egl_surf);
    eglDestroyContext(s_egl_dpy, s_egl_ctx);
    eglTerminate(s_egl_dpy);
    s_egl_dpy = EGL_NO_DISPLAY;
    s_egl_ctx = EGL_NO_CONTEXT;
    s_egl_surf = EGL_NO_SURFACE;
  }
  s_cap_n = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_triangulate
// ─────────────────────────────────────────────────────────────────────────────

int gpu_triangulate(const float* pts_n, int n, const float R[9], const float t[3], float* X_out) {
  if (n <= 0)
    return 0;

  resizeSSBOs(n);

  /* Upload pts_n (4 floats per point) */
  uploadBuf(s_ssbo_pts_in, pts_n, (GLsizeiptr)(4 * n * sizeof(float)));

  /* Upload cam: R[9] + t[3] */
  float cam[12];
  memcpy(cam, R, 9 * sizeof(float));
  memcpy(cam + 9, t, 3 * sizeof(float));
  uploadBuf(s_ssbo_cam, cam, sizeof(cam));

  /* Bind SSBOs */
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, s_ssbo_pts_in);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, s_ssbo_cam);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, s_ssbo_out_tri);

  /* Dispatch */
  glUseProgram(s_prog_tri);
  glUniform1i(glGetUniformLocation(s_prog_tri, "u_n"), n);
  int groups = (n + s_local_size - 1) / s_local_size;
  glDispatchCompute((GLuint)groups, 1, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  /* Download results: float[4n] = [X, Y, Z, valid] */
  std::vector<float> raw(4 * n);
  downloadBuf(s_ssbo_out_tri, raw.data(), (GLsizeiptr)(4 * n * sizeof(float)));

  int valid = 0;
  for (int i = 0; i < n; ++i) {
    X_out[i * 3 + 0] = raw[i * 4 + 0];
    X_out[i * 3 + 1] = raw[i * 4 + 1];
    X_out[i * 3 + 2] = raw[i * 4 + 2];
    if (raw[i * 4 + 3] > 0.5f)
      ++valid;
  }
  return valid;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_ba_residuals
// ─────────────────────────────────────────────────────────────────────────────

void gpu_ba_residuals(const float* pts_px, const float* X, int n, const float R[9],
                      const float t[3], float f, float cx, float cy, float huber_k,
                      float* residuals_out, float* wrss_out, int* valid_out) {
  float wrss = 0.0f;
  int valid = 0;

  if (n <= 0) {
    if (wrss_out)
      *wrss_out = 0.0f;
    if (valid_out)
      *valid_out = 0;
    return;
  }

  resizeSSBOs(n);

  /* Upload inputs */
  uploadBuf(s_ssbo_pts_in, pts_px, (GLsizeiptr)(4 * n * sizeof(float)));
  uploadBuf(s_ssbo_X, X, (GLsizeiptr)(3 * n * sizeof(float)));

  float cam[12];
  memcpy(cam, R, 9 * sizeof(float));
  memcpy(cam + 9, t, 3 * sizeof(float));
  uploadBuf(s_ssbo_cam, cam, sizeof(cam));

  float intr[4] = {f, cx, cy, huber_k};
  uploadBuf(s_ssbo_intr, intr, sizeof(intr));

  /* Bind SSBOs */
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, s_ssbo_pts_in);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, s_ssbo_X);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, s_ssbo_cam);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, s_ssbo_out_res);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, s_ssbo_intr);

  /* Dispatch */
  glUseProgram(s_prog_res);
  glUniform1i(glGetUniformLocation(s_prog_res, "u_n"), n);
  int groups = (n + s_local_size - 1) / s_local_size;
  glDispatchCompute((GLuint)groups, 1, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  /* Download: float[5n] = [r0, r1, r2, r3, valid] */
  std::vector<float> raw(5 * n);
  downloadBuf(s_ssbo_out_res, raw.data(), (GLsizeiptr)(5 * n * sizeof(float)));

  for (int i = 0; i < n; ++i) {
    float v = raw[i * 5 + 4];
    if (v < 0.5f)
      continue;
    ++valid;
    float r0 = raw[i * 5 + 0], r1 = raw[i * 5 + 1];
    float r2 = raw[i * 5 + 2], r3 = raw[i * 5 + 3];
    wrss += r0 * r0 + r1 * r1 + r2 * r2 + r3 * r3;
    if (residuals_out) {
      residuals_out[i * 4 + 0] = r0;
      residuals_out[i * 4 + 1] = r1;
      residuals_out[i * 4 + 2] = r2;
      residuals_out[i * 4 + 3] = r3;
    }
  }
  if (valid > 0)
    wrss /= (float)valid;

  if (wrss_out)
    *wrss_out = wrss;
  if (valid_out)
    *valid_out = valid;
}
