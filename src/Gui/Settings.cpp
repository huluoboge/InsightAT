#include <QSettings>
#include <QFile>
#include <QDebug>
#include <QString>
#include <QDir>
#include "stlplus3/filesystemSimplified/file_system.hpp"
#include "Settings.h"
#include "Utils.h"

namespace insight{

#define  MAX_RECORDS 50

Settings &Settings::instance()
{
	static Settings singleton; 
	return singleton;
}
bool Settings::createDir()
{
	
	bool ok = stlplus::folder_create(tos(dir()));
	
	if (!ok)
	{
		return false;
	}

	QFile file(settings().recentConfigFile());
	file.open(QIODevice::WriteOnly | QIODevice::Truncate);
	file.close();

	return true;
}

QString Settings::dir()
{
	std::string path = stlplus::folder_home() + "/InsightAT";
	return toqs(path);
}

bool Settings::dirExist()
{
	return stlplus::folder_exists(tos(dir()));
}


Settings::Settings()
{
	QSettings settings(recentConfigFile(), QSettings::IniFormat);
	if (settings.value("maxRecentFileCountValue") <= 0)
	{
		settings.setValue("maxRecentFileCountValue", MAX_RECORDS);
	}

	m_lastSucceedImportImagesPath = dir() +  "/lastSucceedImportImagesPath.ini";
	m_ConfigurationParameterFilePath = dir() + "/configurationParameter.ini";

	QSettings setting("defualtMainWindowLayout.ini", QSettings::IniFormat);
	m_defaultGeometryData = (setting.value("geometry")).toByteArray();
	m_defaultLayoutData = (setting.value("state")).toByteArray();
}


QString Settings::recentConfigFile() const
{
	return dir() + "/RecentFile.ini";
}

QStringList Settings::recentProjects() const
{
	QSettings settings(recentConfigFile(), QSettings::IniFormat);
	QStringList files = settings.value("recentFileList").toStringList();
	return files;
}

void Settings::setRecentProjects(const QStringList &files)
{
	QSettings settings(recentConfigFile(), QSettings::IniFormat);
	settings.setValue("recentFileList", files);
}


void Settings::addProjectToRecent(QString prj)
{
	QStringList prjs = recentProjects();
	prjs.removeAll(prj);
	prjs.push_front(prj);
	setRecentProjects(prjs);
}


QString Settings::recentPath() const
{
	QSettings tempSetting(m_lastSucceedImportImagesPath, QSettings::IniFormat);
	return tempSetting.value("path").toString();
}

void Settings::setRecentPath(const QString &recentPath)
{
	QSettings tempSetting(m_lastSucceedImportImagesPath, QSettings::IniFormat);
	tempSetting.setValue("path",recentPath);
}

QString Settings::shortCutPath()
{
	return dir() + "/shortCutData.ini";
}

QString Settings::recentLastSettingProjectPath()
{
	return dir() + "/lastSettingProjectPath.ini";
}

void Settings::setRecentProjectPath(const QString &path)
{
	QSettings tempSettingIni(recentLastSettingProjectPath(), QSettings::IniFormat);
	tempSettingIni.setValue("lastSettingProjectPath", path);
}

QString Settings::recentProjectPath()
{
	QSettings tempSettingIni(recentLastSettingProjectPath(), QSettings::IniFormat);
	QString temp = tempSettingIni.value("lastSettingProjectPath").toString();
	return temp;
}

QStringList Settings::acceptImageFormats()
{
	QStringList filters;
	filters << "jpg" << "jpeg" << "png" << "tif" << "bmp";
	filters << "JPG" << "JPEG" << "PNG" << "TIF" << "BMP";
	return filters;
}

QString Settings::favoriteCoordinateFile() const
{
	return dir() + "/favoriteCoord.ini";
}

QStringList Settings::favoriteCoordinates() const
{
	QSettings settings(favoriteCoordinateFile(), QSettings::IniFormat);//��ǰĿ¼��INI�ļ�
	QStringList names = settings.value("favoriteCood").toStringList();
	return names;
}

void Settings::setFavoriteCoordinate(const QStringList &names)
{
	QSettings settings(favoriteCoordinateFile(), QSettings::IniFormat);//��ǰĿ¼��INI�ļ�
	settings.setValue("favoriteCood", names);
}


}//name space insight