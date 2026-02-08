#ifndef BASE_GPS_H_
#define BASE_GPS_H_

#include <vector>

#include <Eigen/Core>
#include "common_global.h"
#include <cereal/cereal.hpp>
#include "Coordinates.h"
class OGRSpatialReference;
class OGRCoordinateTransformation;
namespace insight{

// Transform WORD coord to latitute longitute and elevation
// representation and vice versa.

class Datum{

public:

    Datum();
    Datum(double a, double b, double f);
    Datum(int EPSG);
    Datum(const char *WKTstr);
    void setInvalid();

    void setParam(double a, double b, double f);

    bool isOK() const;

    //serailize


    // Semimajor axis.
    double a_;
    // Semiminor axis.
    double b_;
    // Flattening.
    double f_;
    // Numerical eccentricity.
    double e2_;
};

class GPSTransform {
public:

    GPSTransform();
    GPSTransform(const Datum &d);

    void setDatum(const Datum &d);
    //lat long and alt
    std::vector<Eigen::Vector3d> EllToXYZ(
            const std::vector<Eigen::Vector3d>& ell) const;

    std::vector<Eigen::Vector3d> XYZToEll(
            const std::vector<Eigen::Vector3d>& xyz) const;

private:

    Datum _datum;
};

//��-��-������ϵENU ������ֱ������ϵת��
//@see https://baike.baidu.com/item/%E7%AB%99%E5%BF%83%E5%9D%90%E6%A0%87%E7%B3%BB/4542391?fr=aladdin

class ENUTransform
{
public:
    ENUTransform(const Datum &d){ _datum = d; }
    ENUTransform(){ _datum.setInvalid(); }

    void setDatum(const Datum &d){ _datum = d; }

    //base xyz or ellipse lat lon alt is both ok
    //����������Ҫһ�����ɣ���һ��ͨ��datum������
    void setBaseXYZ(double X0, double Y0, double Z0);
    void setBaseEll(double lat, double lon, double alt);

    std::vector<Eigen::Vector3d> XYZToENU(const std::vector<Eigen::Vector3d> &xyzs);
    std::vector<Eigen::Vector3d> ENUToXYZ(const std::vector<Eigen::Vector3d> &enus);
private:
    Datum _datum;

    double X0, Y0, Z0;
    double L0, B0, H0;
};

//����ת��
class CoordTransform
{
public:
    void setFrom(OGRSpatialReference *src);
    void setTo(OGRSpatialReference *dst);

    bool setFrom(int epsg);
    bool setTo(int epsg);

    bool beginTransfrom();
    void endTransfrom();
    bool transfrom(double fromX, double fromY, double fromZ, double &targetX,
                   double &dargetY, double &targetZ);

    bool transform(int n, double *xs, double *ys, double *zs);

    //����_oSourceSRS and _oTargetSRS
    //void destorySpatialReference();
    //ͶӰ����ת���������
    static CoordTransform generatePrjToGeoTransform(const Coordinate &prjCoord, bool *ok = nullptr);

    //�������ת����ͶӰ����
    static CoordTransform generateGeoToPjTransform(const Coordinate &prjCoord, bool *ok = nullptr);
private:
    OGRSpatialReference *_oSourceSRS, *_oTargetSRS;
    OGRCoordinateTransformation *_poCT = 0;
};

/**
 * coordinate: input coordinate
 * vecCoords: input coords,local or geodestic(lon,lat,alt)
 * vecENUs: output enu coordinate
**/
bool toENUCoord(const Coordinate &coordinate,
                const std::vector<Vec3> &vecCoords,
                std::vector<Vec3> &vecENUs,Vec3 &enuCenterLonLatAlt);

}//name space insight
#endif  //BASE_GPS_H_
