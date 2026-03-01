#ifndef COMMON_GLOBAL_H
#define COMMON_GLOBAL_H

//#include <QtCore/qglobal.h>


namespace insight{

/**
 *
 * return value(0,1,2) 0 means ok, 1 means stop, 2 means pause
 *
 **/
typedef int progress_fun(float percent, const char *msg);
}//name space insight
#endif // COMMON_GLOBAL_H
