#ifndef qp_doc_Coordinates_h__
#define qp_doc_Coordinates_h__



#include <string>
#include <vector>
#include "common_global.h"
#include "numeric.h"
class  OGRSpatialReference;

namespace insight{
struct WKTTreeItem
{
	std::string begin;
	std::string text;
	std::string end;
};
struct Coordinate
{
	bool isOK() const;
	std::string EPSGName;
	std::string CoordinateName;
	std::string WKT;
	bool isProject(bool *ok) const;

	int EPSG(bool *ok = nullptr) const;

	static bool coordToSR(const Coordinate &coord, OGRSpatialReference &sr);
};

bool parseCoordinates(std::vector<Coordinate> &coords, const std::string &dataBse);


}//name space insight
#endif // qp_doc_Coordinates_h__