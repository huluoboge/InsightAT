/**
 * @file  qstring_convert.h
 * @brief QString 与 std::string 转换工具（从原 Gui/Utils 迁入）
 */

#ifndef INSIGHTAT_UI_QSTRING_CONVERT_H
#define INSIGHTAT_UI_QSTRING_CONVERT_H

#include <QString>
#include <string>

namespace insight {
namespace ui {

inline QString toqs(const std::string& s) { return QString::fromLocal8Bit(s.c_str()); }

inline std::string tos(const QString& s) { return s.toLocal8Bit().constData(); }

}  // namespace ui
}  // namespace insight

#endif  // INSIGHTAT_UI_QSTRING_CONVERT_H
