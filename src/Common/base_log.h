#pragma once


#include "common_global.h"
#include <memory>
#include <sstream>

namespace insight{

class BaseLog;

extern BaseLog *g_base_log;

/*
    logger interface
    */
//enum LOG_LEVEL {
//    eDebug,
//    eInfo,
//    eWarnning,
//    eError
//};

class LogInterface
{
public:
    virtual void msg(const std::string &level, const  char *ms, const char *file, int line) = 0;
    virtual ~LogInterface(){}
};

struct LOGHelper
{
    LOGHelper(std::shared_ptr<LogInterface> log, const std::string & lv, const char *f, int l)
        :logger(log), level(lv), file(f), line(l) {}

    ~LOGHelper() {
        logger->msg(level, ss.str().c_str(), file, line);
    }
    std::stringstream &operator()() {
        return ss;
    }
private:
    std::shared_ptr<LogInterface> logger;
    std::string  level;
    const char *file;
    int line;
    std::stringstream ss;
};

class BaseLog
{
public:
    BaseLog();
    void setLogger(std::shared_ptr<LogInterface> log) { _logger = log; }
    std::shared_ptr<LogInterface> logger() const { return _logger; }

    static BaseLog *instance();
private:
    std::shared_ptr<LogInterface> _logger;//implement;
};


#define ILOG(LEVEL) \
    insight::LOGHelper(insight::BaseLog::instance()->logger(), #LEVEL,__FILE__, __LINE__)()
#define ICHECK(CONDITION) \
    if(!(CONDITION)) insight::LOGHelper(insight::BaseLog::instance()->logger(), "ERROR",__FILE__, __LINE__)()


}//name space insight
