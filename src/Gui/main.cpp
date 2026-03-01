

#include <QtWidgets/QApplication>
#include <QTextCodec>
#include <QtGlobal>
#include <QFileInfo>
#include <QDebug>
#include <QDir>
#include <QMessageBox>
#include <QStandardPaths>


#include <glog/logging.h>
#include <stlplus3/filesystemSimplified/file_system.hpp>

#include "ImageIO/gdal_utils.h"
#include "Common/string_utils.h"
#include "Common/Project.h"
#include "InsightMapper.h"
#include "Document.h"
#include "Utils.h"
#include "MainWindowFrame.h"
using namespace insight;

insight::MainWindowFrame *theWindow;//global window;
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	a.setApplicationName("Insight AT");
	a.setOrganizationName("Insight Map");
    google::InitGoogleLogging(argv[0]);

	QTextCodec *xcodec = QTextCodec::codecForLocale();
	QString exeDir = xcodec->toUnicode(QByteArray(argv[0]));
	QString BKE_CURRENT_DIR = QFileInfo(exeDir).path();
	QStringList  libpath;
	libpath << BKE_CURRENT_DIR;
	libpath << BKE_CURRENT_DIR + QString::fromLocal8Bit("/platforms");
	libpath << BKE_CURRENT_DIR + QString::fromLocal8Bit("/imageformats");
	libpath << BKE_CURRENT_DIR + QString::fromLocal8Bit("/translations");
	libpath << QApplication::libraryPaths();
	
	QApplication::setLibraryPaths(libpath);
    //Don't use Qt::AA_UseSoftwareOpenGL,
    //Use glew instead
    //QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);
    GdalUtils::InitGDAL();

	std::string dataPath;
	if (argc > 1) {
		dataPath = std::string(argv[1]); 
	}
	else {
		dataPath = tos(a.applicationDirPath());
	}
	std::string gdalDataPath = dataPath + "/data/gdal";

	LOG(INFO) << "gdal data path is " << gdalDataPath;
    if(stlplus::folder_exists(gdalDataPath)){
        GdalUtils::SetDataPath(gdalDataPath);
    }else{
        LOG(ERROR)<<"Can't find gdal data path"<< gdalDataPath;
    }
	std::string configPath = dataPath + "/config";

	SystemConfig::instance().setExePath(argv[0]);
    SystemConfig::instance().setConfigPath(configPath);
	SystemConfig::instance().readSensorDatabase();
	SystemConfig::instance().readCoordinate();
	SystemConfig::instance().maxImages = 100000;

    InsightMapper window;
    theWindow = &window;
    window.showMaximized();
    window.initMdiWindows();
    window.refreshWorkspace();

	return a.exec();
}


