#ifndef EXIF_IO_HPP
#define EXIF_IO_HPP

#include <string>
#include "exif.h"

#include "string_utils.h"
namespace insight{

class Exif_IO
{
  public:
    virtual size_t getWidth() const = 0;

    virtual size_t getHeight() const = 0;

    virtual float getFocal() const = 0;

    virtual std::string getBrand() const = 0;

    virtual std::string getModel() const = 0;

    virtual std::string getLensModel() const = 0;

    /** Open the file for checking and parsing */
    virtual bool open( const std::string & sFileName ) = 0;

    /**Verify if the file has metadata*/
    virtual bool doesHaveExifInfo() const = 0;

    /** Print all data*/
    virtual std::string allExifData() const = 0;

};

struct SimpleExifHeader
{
	std::string ImageDescription;   // Image description
	std::string Make;// Camera manufacturer's name
	std::string Model;                // Camera model
	unsigned short BitsPerSample;     // Number of bits per component
	double FocalLength;               // Focal length of lens in millimeters
	unsigned short FocalLengthIn35mm; // Focal length in 35mm film

	struct Geolocation_t {            // GPS information embedded in file
        double Latitude = 0;                  // Image latitude expressed as decimal
        double Longitude = 0;                 // Image longitude expressed as decimal
        double Altitude = 0;                  // Altitude in meters, relative to sea level
        char AltitudeRef = 0;                 // 0 = above sea level, -1 = below sea level
        double DOP = 0;                       // GPS degree of precision (DOP)
		struct Coord_t {
            double degrees = 0;
            double minutes = 0;
            double seconds = 0;
            char direction = 0;
		} LatComponents, LonComponents;   // Latitude, Longitude expressed in deg/min/sec
	} GeoLocation;

	int width = 0;
	int height = 0;


	SimpleExifHeader()
	{
	}

    SimpleExifHeader(const EXIFInfo &info){
        ImageDescription = info.ImageDescription;
		Make = info.Make;
		Model = info.Model;
        ImageDescription = insight::trim(ImageDescription);
        Make = insight::trim(Make);
        Model = insight::trim(Model);
		BitsPerSample = info.BitsPerSample;
		FocalLength = info.FocalLength;
		FocalLengthIn35mm = info.FocalLengthIn35mm;
		memcpy(&GeoLocation, &info.GeoLocation, sizeof(info.GeoLocation));
		width = info.ImageWidth;
		height = info.ImageHeight;
	}
	// Serialization
	template <class Archive>
	void save(Archive & ar) const
	{
		std::string strImageDescription(ImageDescription);
		std::string strMake(Make);
		std::string strModel(Model);

		ar(strImageDescription, strMake, strModel, BitsPerSample, FocalLength, FocalLengthIn35mm);
		ar(GeoLocation.Altitude, GeoLocation.Longitude, GeoLocation.Altitude);
		ar(GeoLocation.LatComponents.degrees, GeoLocation.LatComponents.minutes, GeoLocation.LatComponents.seconds, GeoLocation.LatComponents.direction);
		ar(GeoLocation.LonComponents.degrees, GeoLocation.LonComponents.minutes, GeoLocation.LonComponents.seconds, GeoLocation.LonComponents.direction);
	}

	template <class Archive>
	void load(Archive & ar)
	{
		std::string strImageDescription;
		std::string strMake;
		std::string strModel;

		ar(strImageDescription, strMake, strModel, BitsPerSample, FocalLength, FocalLengthIn35mm);
		ar(GeoLocation.Altitude, GeoLocation.Longitude, GeoLocation.Altitude);
		ar(GeoLocation.LatComponents.degrees, GeoLocation.LatComponents.minutes, GeoLocation.LatComponents.seconds, GeoLocation.LatComponents.direction);
		ar(GeoLocation.LonComponents.degrees, GeoLocation.LonComponents.minutes, GeoLocation.LonComponents.seconds, GeoLocation.LonComponents.direction);

		ImageDescription = strImageDescription;
		Make = strMake;
		Model = strModel;
		//memcpy(ImageDescription, &strImageDescription[0], strImageDescription.size());
		//memcpy(Make, &strMake[0], strMake.size());
		//memcpy(Model, &strModel[0], strModel.size());
	}
};

}//name space insight
#endif //EXIF_IO_HPP

