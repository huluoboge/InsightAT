#include "Document.h"

#include <QApplication>
#include <QMessageBox>

namespace insight{

MAKE_VERSION(Document, 1)

#ifdef WIN32
__declspec(selectany) Coordinate Document::ProjCoordinate;

__declspec(selectany) Coordinate Document::GeoCoordinate;
#else
Coordinate Document::ProjCoordinate;
Coordinate Document::GeoCoordinate;
#endif

void Document::save()
{
    write(toqs(project().projectFile));
}

QString Document::currentFile() const
{
    return toqs(project().projectFile);
}

void Document::write(const QString& file)
{
    project().projectFile = tos(file);
    project().saveProject();
    _bModify = false;
}

const std::string Document::projectExt = ".iatprj";

void Document::read(const QString& file)
{
    project().openProject(tos(file));
    _bModify = true;
}

bool Document::haveFile() const
{
    return !toqs(project().projectFile).isEmpty();
}

void EngineRequest::onTcpRecv()
{
    if (!_isTcpRecvHeadOk) { // 首先读取头信息
        int64_t recv_len = _socket->bytesAvailable();
        if (recv_len < sizeof(ResponseHead)) { // 不能小于头字节数
            return; // 如果没有读完的话，还会再次进入槽函数
        } else {
            // 读取头信息并标注头信息已经被读取
            _isTcpRecvHeadOk = true;
            _tcpRecvBlock.append(_socket->read(sizeof(ResponseHead))); // 读取头信息到tcpRecvBlock
            ResponseHead* header = (ResponseHead*)_tcpRecvBlock.data();
            _header = *header;
            _tcpRecvBlock.clear();
            _isTcpRecvHeadOk = false;
            response(_header);
        }
    }
}

///////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////

bool SingleEngine::startTask(EngineTaskPtr task)
{
    if (isRunning()) {
        LOG(INFO) << "Engine is busy";
        return false;
    }
    curTask = task;
    auto taskName = task->name;
    QProcess* myProcess = new QProcess;
    curProcess = myProcess;
    myProcess->setProcessChannelMode(QProcess::MergedChannels);
    connect(myProcess, SIGNAL(readyRead()), this, SLOT(onRead()));

    connect(myProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [=](int exitCode, QProcess::ExitStatus exitStatus) {
            if (exitStatus == QProcess::NormalExit) {
                if (exitCode == EXIT_SUCCESS) {
                    QByteArray array;
                    array.append(QString("Finish process") + toqs(taskName));
                    message(array);
                    finished(exitCode);
                } else {
                    QByteArray array;
                    array.append(QString("Failed process") + toqs(taskName));
                    message(array);
                    finished(exitCode);
                }
            } else {
                QByteArray array;
                array.append(QString("Crashed process") + toqs(taskName));
                message(array);
                finished(EXIT_FAILURE);
            }
            myProcess->deleteLater();
            curProcess = nullptr;
            curTask.reset();
        });

    // processList[task->name] = myProcess;
    if (task->type == eAT) {
        EngineTaskAT* atTask = static_cast<EngineTaskAT*>(task.get());

        QString path = qApp->applicationDirPath();
        QString program = makeCommand(path, "main_at_execute");
        QStringList arguments;
        QString folder = toqs(task->folder);
        QString doFeat = atTask->doFeat ? "1" : "0";
        QString doMatch = atTask->doMatch ? "1" : "0";
        QString doAt = atTask->doAT ? "1" : "0";
        arguments << "-i"
                  << folder
                  << "-t"
                  << "acurate"
                  << "-f" << doFeat
                  << "-m" << doMatch
                  << "-a" << doAt;

        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    } else if (task->type == eGCPBA) {
        QString path = QCoreApplication::applicationDirPath();
        QString program = makeCommand(path, "main_sfm_gcp");
        QStringList arguments;
        QString folder = toqs(task->folder);
        arguments << "-i"
                  << folder;
        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    } else if (task->type == eExportCC) {
        QString path = QCoreApplication::applicationDirPath();
        QString program = makeCommand(path, "main_export_to_cc");
        QStringList arguments;
        QString folder = toqs(task->folder);
        QString ccFolder = folder + "/CC";
        arguments << "-i"
                  << folder
                  << "-o"
                  << ccFolder;
        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    } else if (task->type == eCheckAT) {
        QString path = QCoreApplication::applicationDirPath();
        QString program = makeCommand(path, "main_check_project");
        QStringList arguments;
        QString folder = toqs(task->folder);
        QString ccFolder = folder + "/CC";
        arguments << "-i"
                  << folder;
        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    } else if (task->type == eRetirangleBA) {
        QString path = QCoreApplication::applicationDirPath();
        QString program = makeCommand(path, "main_retriangle_sfm");
        QStringList arguments;
        QString folder = toqs(task->folder);
        arguments << "-i"
                  << folder;
        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    } else if (task->type == eRefineBA) {
        QString path = QCoreApplication::applicationDirPath();
        QString program = makeCommand(path, "main_refine_sfm");
        QStringList arguments;
        QString folder = toqs(task->folder);
        arguments << "-i"
                  << folder;
        myProcess->start(program, arguments);
        myProcess->waitForStarted();
        LOG(INFO) << "Start running " << task->name;
    }
    return true;
}

void SingleEngine::onRead()
{
    QProcess* process = qobject_cast<QProcess*>(sender());

    const int receive_buffer = 256;

    int64_t recv_len = process->bytesAvailable();
    if (recv_len < receive_buffer) {
        return; // 如果没有读完的话，还会再次进入槽函数
    } else {
        QByteArray datas = process->readAll();
        message(datas);
    }
}

void SingleEngine::flush()
{
    if (curProcess) {
        if (curProcess && curProcess->state() == QProcess::Running) {
            QByteArray datas = curProcess->readAll();
            message(datas);
        }
    }
}
void SingleEngine::stop()
{
    if (curProcess) {
        if (curProcess && curProcess->state() == QProcess::Running) {
            curProcess->kill();
        }
    }
}
bool SingleEngine::isRunning() const
{
    if (curProcess) {
        if (curProcess && curProcess->state() == QProcess::Running) {
            return true;
        }
    }
    return false;
}

std::string SingleEngine::currentTaskName() const
{
    if (curTask) {
        return curTask->name;
    }
    return "";
}
// void Engine::stop(const std::string &name)
// {
//     auto itr = processList.find(name);
//     if(itr != processList.end()){
//         QProcess *process = itr->second;
//         if(process && process->state() == QProcess::Running){
//             process->kill();
//         }
//     }
// }

// bool Engine::isTaskRunning(const std::string &name)
// {
//     auto itr = processList.find(name);
//     if(itr != processList.end()){
//         QProcess *process = itr->second;
//         if(process && process->state() == QProcess::Running){
//             return true;
//         }
//     }
//     return false;
// }
}//name space insight
