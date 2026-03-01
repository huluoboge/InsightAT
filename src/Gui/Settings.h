#ifndef WATSYSTEM_H
#define WATSYSTEM_H

#include <QString>

#include <string>

#include "InsightATGlobal.h"

namespace insight{
class Settings
{
public:
	static Settings &instance(); 

	static bool dirExist();
	static bool createDir();
	static QString dir();
	static QString cameraParamsPath();
	static QString shortCutPath();
	
	void readUserDatas();

	//����򿪵Ĺ���
	void setRecentProjects(const QStringList &files);
	void addProjectToRecent(QString prj);
	QStringList recentProjects() const ;

	//�����·��
	QString recentPath() const;
	void setRecentPath(const QString &recentPath);

	//����������õ���Ŀ·��
	void setRecentProjectPath(const QString &path);
	QString recentProjectPath();

	QString &configurationParameterFilePath() { return m_ConfigurationParameterFilePath; }

	QByteArray defaultGeometryData(){ return m_defaultGeometryData; }
	QByteArray defaultLayoutData(){ return m_defaultLayoutData; }

	QStringList acceptImageFormats();

	QStringList favoriteCoordinates() const;
	void setFavoriteCoordinate(const QStringList &names);

private:
	QString recentConfigFile() const;
	QString recentLastSettingProjectPath(); //��¼�ϴ�������Ŀ·����λ�õ������ļ�

	QString favoriteCoordinateFile() const;
	QString m_lastSucceedImportImagesPath;
	QString m_ConfigurationParameterFilePath;

	QByteArray m_defaultLayoutData;
	QByteArray m_defaultGeometryData;


private:
	Settings();
	Settings(const Settings &);
	Settings &operator=(const Settings &);
	
};

inline Settings &settings(){ return Settings::instance(); }

}//name space insight
#endif