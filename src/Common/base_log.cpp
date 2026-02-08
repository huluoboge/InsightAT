#include "base_log.h"

#include <mutex>
#include <iostream>
#include <string>

namespace insight{

namespace {
    std::mutex g_base_log_mutex;
}

BaseLog *g_base_log = nullptr;

/*
    print to stdout ,default logger
    */
class StdLog : public LogInterface
{
public:
    virtual void msg(const std::string &level, const char *ms, const char *file, int line)
    {
        std::cout << "[" << level << "]" << file << ":" << line << ":"<<ms << std::endl;
    }
};

BaseLog::BaseLog()
{
    _logger.reset(new StdLog);// default std log
}

BaseLog * BaseLog::instance()
{
    if (g_base_log != nullptr) {
        return g_base_log;
    }
    else {
        std::lock_guard<std::mutex> guard(g_base_log_mutex);
        if (g_base_log == nullptr) {
            g_base_log = new BaseLog;
        }
        return g_base_log;
    }
}


}//name space insight
