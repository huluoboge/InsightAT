#include "data_sheet.h"


namespace insight{

 bool parseDatabase(const std::string& sfileDatabase, std::vector<Datasheet>& vec_database)
{
	bool createDatabase = false;
	std::ifstream iFilein(sfileDatabase.c_str());
	if (iFilein.is_open())
	{
		std::string line;
		while (iFilein.good())
		{
			getline(iFilein, line);
			if (!line.empty())
			{
				//std::stringstream sStream( line );
				if (line[0] != '#')
				{
					std::vector<std::string> values;
					split(line, ";", values);
					if (values.size() == 3)
					{
						const std::string brand = values[0];
						const std::string model = values[1];
						const double sensorSize = atof(values[2].c_str());
						vec_database.push_back(Datasheet(brand, model, sensorSize));
					}
				}
			}
		}
		createDatabase = true;
	}
	else
	{
		CHECK(false)<< "Cannot open the database file: "<< sfileDatabase << std::endl;
	}

	return createDatabase;
}

 bool getInfo(const std::string& sBrand, const std::string& sModel, const std::vector<Datasheet>& vec_database, Datasheet& datasheetContent)
{
	bool existInDatabase = false;

	Datasheet refDatasheet(sBrand, sModel, -1.);
	std::vector<Datasheet>::const_iterator datasheet = std::find(vec_database.begin(), vec_database.end(), refDatasheet);
	if (datasheet != vec_database.end())
	{
		datasheetContent = *datasheet;
		existInDatabase = true;
	}

	return existInDatabase;
}

bool Datasheet::operator==(const Datasheet& ds) const
{
	bool isEqual = false;
	std::vector<std::string> vec_brand;
	split(ds._brand, " ", vec_brand);
	std::string brandlower = _brand;

	for (int index = 0; index < brandlower.length(); index++)
	{
		brandlower[index] = tolower(brandlower[index]);
	}

	for (std::vector<std::string>::const_iterator iter_brand = vec_brand.begin();
		iter_brand != vec_brand.end();
		iter_brand++)
	{
		std::string brandlower2 = *iter_brand;
		for (int index = 0; index < brandlower2.length(); index++)
		{
			brandlower2[index] = tolower(brandlower2[index]);
		}
		//std::cout << brandlower << "\t" << brandlower2 << std::endl;
		if (brandlower.compare(brandlower2) == 0)
		{

			std::vector<std::string> vec_model1;
			split(ds._model, " ", vec_model1);
			std::vector<std::string> vec_model2;
			split(_model, " ", vec_model2);
			bool isAllFind = true;
			for (std::vector<std::string>::const_iterator iter_model1 = vec_model1.begin();
				iter_model1 != vec_model1.end();
				iter_model1++)
			{
				bool hasDigit = false;
				for (std::string::const_iterator c = (*iter_model1).begin(); c != (*iter_model1).end(); ++c)
				{
					if (isdigit(*c))
					{
						hasDigit = true;
						break;
					}
				}
				if (hasDigit)
				{
					std::string modellower1 = *iter_model1;
					for (int index = 0; index < modellower1.length(); index++)
					{
						modellower1[index] = tolower(modellower1[index]);
					}
					bool isFind = false;
					for (std::vector<std::string>::const_iterator iter_model2 = vec_model2.begin();
						iter_model2 != vec_model2.end();
						iter_model2++)
					{
						std::string modellower2 = *iter_model2;
						for (int index = 0; index < modellower2.length(); index++)
						{
							modellower2[index] = tolower(modellower2[index]);
						}
						if (modellower2.compare(modellower1) == 0)
						{
							isFind = true;
						}
					}
					if (!isFind)
					{
						isAllFind = false;
						break;
					}
				}
			}
			if (isAllFind)
				isEqual = true;
		}
	}
	return isEqual;
}

}//name space insight