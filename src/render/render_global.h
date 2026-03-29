/**
 * @file  render_global.h
 * @brief 渲染库全局配置：导出宏、OpenGL 头文件引入及编译器警告抑制。
 *
 * Architecture
 * ─────────────
 *  本文件是 src/render/ 库的全局依赖入口，位于模块层次的最底层。
 *  所有其他渲染库头文件均应将本文件置于项目内部头文件组的首位引入。
 *  依赖关系：render_global.h ← 所有 render_*.h
 *
 * Usage
 * ─────
 *   // 在任意渲染库头文件中：
 *   #include "render/render_global.h"
 *
 *   class RENDER_EXPORT MyRenderClass { ... };
 */

#pragma once
#ifndef RENDER_GLOBAL_H
#define RENDER_GLOBAL_H

// ── C++ 标准库 ────────────────────────────────────────────────────────────────

// ── 第三方库（Qt） ────────────────────────────────────────────────────────────
#include <QtCore/qglobal.h>

// ── 第三方库（OpenGL / GLEW） ─────────────────────────────────────────────────
#if defined(WIN32) || defined(WIN64) || defined(_WIN32) || defined(_WIN64)
#include "gl/glew.h"
#else
#include "GL/glew.h"
#endif

// ── 编译器警告抑制 ────────────────────────────────────────────────────────────
#pragma warning(disable : 4251)

// ── 导出宏定义 ────────────────────────────────────────────────────────────────
#ifndef BUILD_STATIC
#define BUILD_STATIC
#endif // BUILD_STATIC

#ifndef BUILD_STATIC
#if defined(RENDER_LIB)
#define RENDER_EXPORT Q_DECL_EXPORT
#else
#define RENDER_EXPORT Q_DECL_IMPORT
#endif
#else
#define RENDER_EXPORT
#endif

#endif // RENDER_GLOBAL_H
