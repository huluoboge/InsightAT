#include <iostream>
#include <fstream>
#include <sstream>
#include "Coordinates.h"
#include "stlplus3/filesystemSimplified/file_system.hpp"
#include "string_utils.h"
#include "gdal.h"
#include <ogr_spatialref.h>
#include <sstream>

namespace insight{

bool parseCoordinates(std::vector<Coordinate> &coords, const std::string &dataBse)
{
	if (!stlplus::file_exists(dataBse))
	{
		return false;
	}
	coords.clear();
	std::ifstream ifs(dataBse);

	char *buffer = new char[90000];
	coords.reserve(7000);
	while (ifs.getline(buffer, 90000))
	{
		std::vector<std::string> values;
		split(buffer, ";", values);
		if (values.size() == 3)
		{
			Coordinate c;
			c.EPSGName = values[0];
			c.CoordinateName = values[1];
			c.WKT = values[2];
			coords.push_back(c);
		}
	}
	delete[] buffer;
	return true;
}

bool insight::Coordinate::isOK() const
{
	if (EPSGName.empty())return false;
	OGRSpatialReference oPGS;
	std::stringstream ss;
	ss.str(EPSGName);
	int epsg = 0;
	ss >> epsg;
	int error = oPGS.importFromEPSG(epsg);
	if (error != OGRERR_NONE){
		return false;
	}
	return true;
}

bool Coordinate::isProject(bool *ok) const
{
	OGRSpatialReference oPGS;
	std::stringstream ss;
	ss.str(EPSGName);
	int epsg = 0;
	ss >> epsg;
	int error = oPGS.importFromEPSG(epsg);
	if (error != OGRERR_NONE){
		LOG(ERROR) << "Can't import epsg " << epsg;
		if(ok) *ok = false;
		return false;
	}
	if (ok) *ok = true;
	if (oPGS.IsProjected()){
		return true;
	}
	else{
		return false;
	}
}


int Coordinate::EPSG(bool *ok) const
{
	if (EPSGName.empty()){
		if (ok != nullptr){
			*ok = false;
		}
		return 0;
	}
	std::string s = EPSGName;
	s = trim(s);
	int epsg = 0;
	std::stringstream ss(s);
	ss >> epsg;
	return epsg;
}
bool Coordinate::coordToSR(const Coordinate &coord, OGRSpatialReference &sr)
{
	bool ok = false;
	int fromEPSG = coord.EPSG(&ok);
	if (ok){
		OGRErr err = sr.importFromEPSG(fromEPSG);
		if (err != OGRERR_NONE){
			printf("Can't import coord with epsg = %d\n", fromEPSG);
			return false;
		}
	}
	else if (!coord.WKT.empty()){
		char buf[8192] = { 0 };
		const char *pBuf = buf;
		memcpy(buf, coord.WKT.c_str(), coord.WKT.size());
		OGRErr err = sr.importFromWkt((const char**)&pBuf);
		if (err != OGRERR_NONE){
			printf("Can't import coord with wkt = %s\n", coord.WKT.c_str());
			return false;
		}
	}
	else{
		printf("Can't import coord \n");
		return false;
	}
	return true;
}


}//name space insight


