
#ifndef TASK_LIST_DATA_SHEET_H
#define TASK_LIST_DATA_SHEET_H


#include "common.h"

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>

#include "common_global.h"
#include "glog/logging.h"


namespace insight{
// Database structure
struct  Datasheet
{
	Datasheet()
	{}

	Datasheet(const std::string& brand,
		const std::string& model,
		const double& sensorSize) :
		_brand(brand),
		_model(model),
		_sensorSize(sensorSize)
	{}

	bool operator==(const Datasheet& ds) const;

	std::string _brand;
	std::string _model;
	double _sensorSize;

	template <class Archive>
	void save(Archive & ar) const
	{
		ar(_brand, _model, _sensorSize);
	}

	template <class Archive>
	void load(Archive & ar)
	{
		ar(_brand, _model, _sensorSize);
	}
};

// Parse the database
 bool parseDatabase(const std::string& sfileDatabase, std::vector<Datasheet>& vec_database);

// Get information for the given camera model
 bool getInfo(const std::string& sBrand, const std::string& sModel, const std::vector<Datasheet>& vec_database, Datasheet& datasheetContent);

}//name space insight

#endif // TASK_LIST_DATA_SHEET_H




