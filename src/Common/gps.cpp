#include "gps.h"
#include "blocks.h"
#include "numeric.h"
#include <Eigen/Core>
#include <cmath>
#include <gdal.h>
#include <limits>
#include <ogr_spatialref.h>

#include <sstream>
namespace insight{

Datum::Datum(const int EPSG)
{
    OGRSpatialReference oPGS;
    int error = oPGS.importFromEPSG(EPSG);
    if (error != OGRERR_NONE) {
        LOG(ERROR) << "Can't import epsg " << EPSG;
        setInvalid();
        return;
    }
    double a = oPGS.GetSemiMajor();
    double b = oPGS.GetSemiMinor();
    double f = 1.0 / oPGS.GetInvFlattening();
    setParam(a, b, f);
}

Datum::Datum()
{
    setInvalid();
}

Datum::Datum(double a, double b, double f)
{
    setParam(a, b, f);
}

Datum::Datum(const char* WKTstr)
{
    OGRSpatialReference oPGS;
    int error = oPGS.importFromWkt(&WKTstr);
    if (error != OGRERR_NONE) {
        LOG(ERROR) << "Can't import WKT " << WKTstr;
        setInvalid();
        return;
    }
    double a = oPGS.GetSemiMajor();
    double b = oPGS.GetSemiMinor();
    double f = 1.0 / oPGS.GetInvFlattening();
    setParam(a, b, f);
}

void Datum::setInvalid()
{
    a_ = std::numeric_limits<double>::quiet_NaN();
    b_ = std::numeric_limits<double>::quiet_NaN();
    f_ = std::numeric_limits<double>::quiet_NaN();
}

void Datum::setParam(double a, double b, double f)
{
    a_ = a;
    b_ = b;
    f_ = f;
    e2_ = (a_ * a_ - b_ * b_) / (a_ * a_);
}

bool Datum::isOK() const
{
    return (a_ != std::numeric_limits<double>::quiet_NaN() && b_ != std::numeric_limits<double>::quiet_NaN() && f_ != std::numeric_limits<double>::quiet_NaN());
}

GPSTransform::GPSTransform(const Datum& d)
{
    _datum = d;
}

GPSTransform::GPSTransform()
{
    _datum.setInvalid();
}

void GPSTransform::setDatum(const Datum& d)
{
    _datum = d;
}

std::vector<Eigen::Vector3d> GPSTransform::EllToXYZ(
    const std::vector<Eigen::Vector3d>& ell) const
{
    std::vector<Eigen::Vector3d> xyz(ell.size());

    double a_ = _datum.a_;
    double b_ = _datum.b_;
    double f_ = _datum.f_;
    double e2_ = _datum.e2_;

    for (size_t i = 0; i < ell.size(); ++i) {
        const double lat = D2R(ell[i](0));
        const double lon = D2R(ell[i](1));
        const double alt = ell[i](2);

        const double sin_lat = sin(lat);
        const double sin_lon = sin(lon);
        const double cos_lat = cos(lat);
        const double cos_lon = cos(lon);

        // Normalized radius
        const double N = a_ / sqrt(1 - e2_ * sin_lat * sin_lat);

        xyz[i](0) = (N + alt) * cos_lat * cos_lon;
        xyz[i](1) = (N + alt) * cos_lat * sin_lon;
        xyz[i](2) = (N * (1 - e2_) + alt) * sin_lat;
    }

    return xyz;
}

std::vector<Eigen::Vector3d> GPSTransform::XYZToEll(
    const std::vector<Eigen::Vector3d>& xyz) const
{
    std::vector<Eigen::Vector3d> ell(xyz.size());

    double a_ = _datum.a_;
    double e2_ = _datum.e2_;

    for (size_t i = 0; i < ell.size(); ++i) {
        const double x = xyz[i](0);
        const double y = xyz[i](1);
        const double z = xyz[i](2);

        const double xx = x * x;
        const double yy = y * y;

        const double kEps = 1e-12;

        // Latitude
        double lat0 = atan2(z, sqrt(xx + yy));
        double lat = 100;
        double alt = 0;

        for (size_t j = 0; j < 100; ++j) {
            const double sin_lat0 = sin(lat0);
            const double N = a_ / sqrt(1 - e2_ * sin_lat0 * sin_lat0);
            alt = sqrt(xx + yy) / cos(lat0) - N;
            lat = lat0;
            lat0 = atan2(z / (N * (1 - e2_) + alt), sqrt(xx + yy) / (N + alt));
            // lat0 = atan((z / sqrt(xx + yy)) * 1 / (1 - e2_ * N / (N + alt)));

            if (std::fabs(lat - lat0) < kEps) {
                break;
            }
        }

        ell[i](0) = R2D(lat0);

        // Longitude
        ell[i](1) = R2D(atan2(y, x));
        // Alt
        ell[i](2) = alt;
    }

    return ell;
}

void ENUTransform::setBaseXYZ(double x0, double y0, double z0)
{
    GPSTransform trans;
    trans.setDatum(_datum);
    std::vector<Eigen::Vector3d> baseXYZ;
    baseXYZ.push_back(Eigen::Vector3d(x0, y0, z0));
    Eigen::Vector3d ell = trans.XYZToEll(baseXYZ)[0];
    X0 = x0;
    Y0 = y0;
    Z0 = z0;
    B0 = ell[0];
    L0 = ell[1];
    H0 = ell[2];
}

void ENUTransform::setBaseEll(double _lat, double _lon, double _alt)
{
    B0 = _lat;
    L0 = _lon;
    H0 = _alt;
    GPSTransform trans;
    trans.setDatum(_datum);
    std::vector<Eigen::Vector3d> ell;
    ell.push_back(Eigen::Vector3d(B0, L0, H0));
    Eigen::Vector3d xyz = trans.EllToXYZ(ell)[0];
    X0 = xyz[0];
    Y0 = xyz[1];
    Z0 = xyz[2];
}

std::vector<Eigen::Vector3d> ENUTransform::XYZToENU(
    const std::vector<Eigen::Vector3d>& xyzs)
{
    std::vector<Eigen::Vector3d> enus;
    const double lat = D2R(B0);
    const double lon = D2R(L0);
    //	const double alt = H0;
    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);

    for (size_t i = 0; i < xyzs.size(); ++i) {
        double X = xyzs[i].x();
        double Y = xyzs[i].y();
        double Z = xyzs[i].z();
        double E = -(X - X0) * sin_lon + (Y - Y0) * cos_lon;
        double N = -(X - X0) * sin_lat * cos_lon - (Y - Y0) * sin_lat * sin_lon + (Z - Z0) * cos_lat;
        double U = (X - X0) * cos_lat * cos_lon + (Y - Y0) * cos_lat * sin_lon + (Z - Z0) * sin_lat;
        enus.push_back(Eigen::Vector3d(E, N, U));
    }
    return enus;
}

std::vector<Eigen::Vector3d> ENUTransform::ENUToXYZ(const std::vector<Eigen::Vector3d>& enus)
{
    const double lat = D2R(B0);
    const double lon = D2R(L0);
    //	const double alt = H0;
    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);
    std::vector<Eigen::Vector3d> xyzs;
    double X, Y, Z;
    for (size_t i = 0; i < enus.size(); ++i) {
        double E = enus[i].x();
        double N = enus[i].y();
        double U = enus[i].z();
        X = -E * sin_lon - N * sin_lat * cos_lon + U * cos_lat * cos_lon + X0;
        Y = E * cos_lon - N * sin_lat * sin_lon + U * cos_lat * sin_lon + Y0;
        Z = N * cos_lat + U * sin_lat + Z0;
        xyzs.push_back(Eigen::Vector3d(X, Y, Z));
    }
    return xyzs;
}

void CoordTransform::setFrom(OGRSpatialReference* src)
{
    _oSourceSRS = src;
}

bool CoordTransform::setFrom(int epsg)
{
    _oSourceSRS = new OGRSpatialReference;
    if (OGRERR_NONE == _oSourceSRS->importFromEPSG(epsg)) {
        return true;
    }
    return false;
}

void CoordTransform::setTo(OGRSpatialReference* dst)
{
    _oTargetSRS = dst;
}

bool CoordTransform::setTo(int epsg)
{
    _oTargetSRS = new OGRSpatialReference;
    if (OGRERR_NONE == _oTargetSRS->importFromEPSG(epsg)) {
        return true;
    }
    return false;
}

bool CoordTransform::beginTransfrom()
{
    if (_poCT) {
        endTransfrom();
    }
    _poCT = OGRCreateCoordinateTransformation(_oSourceSRS,
        _oTargetSRS);
    if (_poCT == nullptr) {
        printf("Can't transform from coord");
        return false;
    }
    return true;
}

void CoordTransform::endTransfrom()
{
    OGRCoordinateTransformation::DestroyCT(_poCT);
    _poCT = nullptr;
}
bool CoordTransform::transfrom(double fromX, double fromY, double fromZ, double& targetX, double& targetY, double& targetZ)
{
    if (_poCT == nullptr)
        return false;
    if (TRUE == _poCT->Transform(1, &fromX, &fromY, &fromZ)) {
        targetX = fromX;
        targetY = fromY;
        targetZ = fromZ;
        return true;
    } else {
        return false;
    }
}

bool CoordTransform::transform(int n, double* xs, double* ys, double* zs)
{
    if (_poCT == nullptr)
        return false;
    if (TRUE == _poCT->Transform(n, xs, ys, zs)) {
        return true;
    } else {
        return false;
    }
}

CoordTransform CoordTransform::generatePrjToGeoTransform(
    const Coordinate& prjCoord, bool* ok)
{
    CoordTransform t;
    if (prjCoord.isProject(ok)) {
        OGRSpatialReference* src = new OGRSpatialReference;
        bool bOK = Coordinate::coordToSR(prjCoord, *src);

        if (bOK) {
            OGRSpatialReference* dst = src->CloneGeogCS();
            t.setFrom(src);
            t.setTo(dst);
            return t;
        } else {
            if (ok) {
                *ok = false;
            }
            return t;
        }
    } else {
        if (ok) {
            *ok = false;
        }
        return t;
    }
}

CoordTransform CoordTransform::generateGeoToPjTransform(
    const Coordinate& prjCoord, bool* ok /*= nullptr*/)
{
    CoordTransform t;
    if (prjCoord.isProject(ok)) {
        OGRSpatialReference* dst = new OGRSpatialReference;
        bool bOK = Coordinate::coordToSR(prjCoord, *dst);

        if (bOK) {
            OGRSpatialReference* src = dst->CloneGeogCS();
            t.setFrom(src);
            t.setTo(dst);
            return t;
        } else {
            if (ok) {
                *ok = false;
            }
            return t;
        }
    } else {
        if (ok) {
            *ok = false;
        }
        return t;
    }
}

bool toENUCoord(const Coordinate& coordinate,
    const std::vector<Vec3>& vecCoords,
    std::vector<Vec3>& vecENUs,
    Vec3& enuCenterLonLatAlt)
{
    if (vecCoords.empty()) {
        LOG(INFO) << "the input coord is empty";
        return false;
    }
    OGRSpatialReference sr;
    bool ok = Coordinate::coordToSR(coordinate, sr);
    if (!ok) {
        LOG(ERROR) << "can't convert to spatial reference,the epsg or wkt many has error";
        return false;
    }

    double a = sr.GetSemiMajor();
    double b = sr.GetSemiMinor();
    double f = 1.0 / sr.GetInvFlattening();
    Datum dtum(a, b, f);

    std::vector<Vec3> ell; // lat lon alt
    if (sr.IsProjected()) {
        // prj->geo->world->enu
        CoordTransform transform = CoordTransform::generatePrjToGeoTransform(coordinate, &ok);
        if (!ok) {
            LOG(ERROR) << "can't convert PRJ to GEO";
            return false;
        }
        std::vector<double> xs, ys, zs;
        for (auto v : vecCoords) {
            xs.emplace_back(v.x());
            ys.emplace_back(v.y());
            zs.emplace_back(v.z());
        }

        Vec3 center = vecCoords[0];

        // PRJ->GEO
        int n = xs.size();
        transform.beginTransfrom();
        transform.transform(n, xs.data(), ys.data(), zs.data());
        transform.endTransfrom();
        for (int i = 0; i < xs.size(); ++i) {
            // lat,lon,alt
            ell.emplace_back(ys[i], xs[i], zs[i]);
        }
        Vec3 centerLonLatAlt = center;
        // transform center to log lat alt
        transform.beginTransfrom();
        transform.transform(1, &centerLonLatAlt.x(), &centerLonLatAlt.y(), &centerLonLatAlt.z());
        transform.endTransfrom();
        enuCenterLonLatAlt = centerLonLatAlt;
    } else {
        for (auto v : vecCoords) {
            // lat lon alt
            ell.emplace_back(v.y(), v.x(), v.z());
        }
        // TODO get the center
        enuCenterLonLatAlt = Vec3(vecCoords[0]);
    }
    // GEO->WORLD

    ENUTransform trans(dtum);
    trans.setBaseEll(enuCenterLonLatAlt.y(), enuCenterLonLatAlt.x(), enuCenterLonLatAlt.z());
    GPSTransform worldTrans(dtum);
    std::vector<Vec3> xyz;
    xyz = worldTrans.EllToXYZ(ell);
    // WORLD->GEO
    vecENUs = trans.XYZToENU(xyz);
    return true;
}

}//name space insight
