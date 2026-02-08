#ifndef Document_h__
#define Document_h__

#include "Common/Coordinates.h"
#include "Common/Project.h"
#include "InsightATGlobal.h"

#include <stdint.h>
#include <string>

#include <QObject>
#include <QString>

#include "Utils.h"
#include "stlplus3/filesystemSimplified/file_system.hpp"

#include <QByteArray>
#include <QDataStream>
#include <QHostAddress>
#include <QObject>
#include <QProcess>
#include <QSettings>
#include <QTcpSocket>
#include <QThread>
#include <iostream>
#include <map>

#ifndef DECLARE_VERSION
#define DECLARE_VERSION \
    static const int VERSION;

#define MAKE_VERSION(CLASS, VER) \
    const int CLASS::VERSION = VER;

#endif

namespace insight{

inline Project& project()
{
    static Project prj;
    return prj;
}

struct Document : public QObject {
    Q_OBJECT
public:
    enum OPEN_MODE {
        eOpen,
        eClose
    };

    static Coordinate ProjCoordinate;
    static Coordinate GeoCoordinate;

    static const std::string projectExt;
signals:
    void modifying();
    void openStateChanged();

public:
public slots:
    void read(const QString& file);
    void write(const QString& file);
    void save();

    QString currentFile() const;

    void setModify(bool modify)
    {
        _bModify = modify;
        emit modifying();
    }

    void setOpen(bool val)
    {
        val ? open_mode = eOpen : open_mode = eClose;
        emit openStateChanged();
    }

    bool isOpen() const
    {
        return (open_mode == eOpen);
    }
    bool isModified() const { return _bModify; }
    bool haveFile() const;

public:
    DECLARE_VERSION

private:
    bool _bModify = false;
    OPEN_MODE open_mode = eClose;
};

inline Document& doc()
{
    static Document document;
    return document;
}

//task defines
enum TaskStatus {
    eCreated,
    eAppending,
    eRunning,
    eFinished,
    eCanceled,
};

enum TaskType {
    eUnknown,
    eAT,
    eModel,
    eGCPBA,
    eExportCC,
    eCheckAT,
    eRetirangleBA,
    eRefineBA,
};

struct EngineTask {
    int type = eUnknown;
    std::string name;
    std::string folder;
    int status = eCreated; //TaskStatus
};

struct EngineTaskAT : public EngineTask {
    EngineTaskAT()
    {
        type = eAT;
        status = eCreated;
    }
    bool doFeat = true;
    bool doMatch = true;
    bool doAT = true;
};

typedef std::shared_ptr<EngineTask> EngineTaskPtr;
typedef std::shared_ptr<EngineTaskAT> EngineTaskATPtr;

enum CmdType {
    eAdd,
    eModify,
    eQuery,
};

enum QueryType {
    eQueryStatus = 1, //query the task status
    eQueryProgress = 2, //query the task progress
};

class RequestHead {
public:
    RequestHead()
    {
        memset(checkData, 0, 256);
        memset(name, 0, 256);
        memset(taskPath, 0, 256);
        std::string check64 = GetUUID();
        sprintf(checkData, "%s", check64.c_str());
    }
    char checkData[256]; //check by sorket

    char name[256]; //task id
    int commandType = 0; //add= 0,modify=1,query=2
    int addTaskType; //eAt,eModel
    int modifyToStatus = -1; //TaskStatus
    int queryType = -1; //if commandType=query,set the queryType to queryStatus(1)
    //params
    char taskPath[2048]; //task full working path.(optional,only used by add)
    //datas
    int dataBytes = 0; //orther datas
};

class ResponseHead {
public:
    ResponseHead()
    {
        memset(checkData, 0, 256);
        memset(errorMsg, 0, 256);
    }
    char checkData[256];
    char errorMsg[256];
    int result = -1; //0 = OK

    int queryStatus = -1; //if query type is status, return the status;
    float progress = 0; //if query type is progress,return the progress
};

class EngineRequest : public QObject {
    Q_OBJECT
signals:
    void response(const ResponseHead&);

public:
    EngineRequest(QObject* parent = nullptr)
        : QObject(parent)
    {
        _isTcpRecvHeadOk = false;
        _socket = new QTcpSocket;
        QObject::connect(_socket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(displayError(QAbstractSocket::SocketError)));
        QObject::connect(_socket, SIGNAL(readyRead()), this, SLOT(onTcpRecv()));
    }
    ~EngineRequest()
    {
        _socket->close();
        delete _socket;
    }

public slots:
    void send(const RequestHead& request)
    {
        QByteArray outputdata;
        outputdata.resize(sizeof(RequestHead));
        memcpy(outputdata.data(), &request, sizeof(RequestHead));

        _socket->write(outputdata);
        _socket->waitForBytesWritten();
    }

    void displayError(QAbstractSocket::SocketError error)
    {
        qDebug() << _socket->errorString();
    }

    void onTcpRecv();

    bool connect()
    {
        if (isConnected())
            return true;
        QSettings setting("SocketConfig.ini");
        int port = setting.value("port", 7777).toInt();
        _socket->connectToHost(QHostAddress(QHostAddress::LocalHost), port);
        if (!_socket->waitForConnected()) {
            printf("Can't connect server on port: %d\n", port);
            return false;
        }
        return true;
    }

    bool isConnected()
    {
        if (_socket->state() == QAbstractSocket::ConnectedState) {
            return true;
        }
        return false;
    }

private:
    QTcpSocket* _socket;
    bool _isTcpRecvHeadOk;
    QByteArray _tcpRecvBlock;
    ResponseHead _header;
};

class SingleEngine : public QObject {
    Q_OBJECT
signals:
    void message(const QByteArray&);
    void finished(int exitCode);

public:
    static SingleEngine& instance()
    {
        static SingleEngine single;
        return single;
    }

public slots:
    bool startTask(EngineTaskPtr task);
    void stop();
    bool isRunning() const;
    std::string currentTaskName() const;
    // void stop(const std::string &name);
    // bool isTaskRunning(const std::string &name);
    void flush();
private slots:
    void onRead();

private:
    SingleEngine() { }
    ~SingleEngine() { }

    QString makeCommand(const QString& path, const QString& program)
    {
#ifdef WIN32
        const QString ext = ".exe";
#else
        const QString ext = "";
#endif
        return path + "/" + program + ext;
    }

    EngineTaskPtr curTask;
    QProcess* curProcess;
    // std::map<std::string, QProcess *> processList;
};

}//name space insight
#endif // Document_h__
