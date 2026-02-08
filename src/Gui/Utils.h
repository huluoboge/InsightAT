#ifndef Utils_h__
#define Utils_h__

#include <QString>
#include <string>
#include <QProcess>
#include "stlplus3/filesystemSimplified/file_system.hpp"

#include "InsightATGlobal.h"
namespace insight{
inline QString toqs(const std::string &s)
{
	return QString::fromLocal8Bit(s.c_str());
}

inline std::string tos(const QString &s)
{
	return s.toLocal8Bit().constData();
}



}//name space insight
#endif // Utils_h__

