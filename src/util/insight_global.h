/**
 * @file insight_global.h
 * @brief 最小全局声明（从原 Common/common_global.h 迁入）
 */

#ifndef INSIGHT_UTIL_GLOBAL_H
#define INSIGHT_UTIL_GLOBAL_H

namespace insight {

/** 进度回调：返回值 0=继续, 1=停止, 2=暂停 */
typedef int progress_fun(float percent, const char* msg);

} // namespace insight

#endif // INSIGHT_UTIL_GLOBAL_H
