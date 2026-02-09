
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdio.h>
#include <unordered_set>
#include <vector>

#include "stlplus3/filesystemSimplified/file_system.hpp"
#include <Eigen/Geometry>
#include <cmdLine/cmdLine.h>

inline double D2R(const double deg)
{
    return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

inline double R2D(double radian)
{
    return radian / M_PI * 180.0;
}

Eigen::Vector3d toXYZ(
    Eigen::Vector3d& lonLatHeight, double a, double invF)
{
    // f = (a-b)/a
    // invf = a/(a-b)
    double b = a - a / invF;
    //  e = sqrt(a*a-b*b)/a
    double sqe = (a * a - b * b) / (a * a);
    Eigen::Vector3d XYZ;

    const double lon = D2R(lonLatHeight.x());
    const double lat = D2R(lonLatHeight.y());
    const double alt = lonLatHeight.z();

    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);

    // Normalized radius
    const double RN = a / sqrt(1 - sqe * sin_lat * sin_lat);

    XYZ.x() = (RN + alt) * cos_lat * cos_lon;
    XYZ.y() = (RN + alt) * cos_lat * sin_lon;
    XYZ.z() = (RN * (1 - sqe) + alt) * sin_lat;
    return XYZ;
}

Eigen::Vector3d fromXYZ(
    Eigen::Vector3d& XYZ, double a, double invF)
{
    Eigen::Vector3d lonLatHeight;
    // f = (a-b)/a
    // invf = a/(a-b)
    double b = a - a / invF;
    //  e = sqrt(a*a-b*b)/a
    double sqe = (a * a - b * b) / (a * a);
    const double sqrt_xx_yy = sqrt(XYZ.x() * XYZ.x() + XYZ.y() * XYZ.y());

    const double kEps = 1e-12;
    // Latitude
    //[-PI,PI]
    lonLatHeight.x() = R2D(atan2(XYZ.y(), XYZ.x()));
    // iterator solver lat and height

    double lat = atan2(XYZ.z(), sqrt_xx_yy);
    double alt = 0;
    const int maxItrTimes = 1000;
    int itr = 0;
    while (itr++ < maxItrTimes) {
        const double sin_lat = sin(lat);
        const double RN = a / sqrt(1 - sqe * sin_lat * sin_lat);
        alt = sqrt_xx_yy / cos(lat) - RN;
        double lat1 = atan2(XYZ.z() / (RN * (1 - sqe) + alt), sqrt_xx_yy / (RN + alt));
        double err = std::fabs(lat1 - lat);
        std::cout << err << ",";
        if (err < kEps) {
            break;
        }
        lat = lat1;
    }
    std::cout << "\n";
    std::cout << "iter total " << itr << " times\n";
    lonLatHeight.y() = R2D(lat);
    lonLatHeight.z() = alt;
    return lonLatHeight;
}

Eigen::Matrix3d RotationOfENU2XYZ(double lonDeg, double latDeg)
{
    const double lon = D2R(lonDeg);
    const double lat = D2R(latDeg);
    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);

    Eigen::Matrix3d rot;
    rot << -sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon,
        cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon,
        0, cos_lat, sin_lat;
    return rot;
}

Eigen::Matrix3d RotationOfXYZ2ENU(double lonDeg, double latDeg)
{
    Eigen::Matrix3d rot = RotationOfENU2XYZ(lonDeg, latDeg);
    return rot.transpose();
}

int main0(int argc, char* argv[])
{
    Eigen::Vector3d XYZ;
    XYZ << -2425635.2999582700431347, 5022042.4671905003488064, 3084219.4893416501581669;

    Eigen::Vector3d lonLatHeight;

    double wgs84_a = 6378137.0;
    double wgs84_invF = 298.257223563;
    lonLatHeight = fromXYZ(XYZ, wgs84_a, wgs84_invF);
    std::cout.precision(10);
    std::cout << std::fixed;
    std::cout << lonLatHeight.transpose() << std::endl;
    return 0;
}

int main1(int argc, char* argv[])
{
    Eigen::Vector3d lonLatHeight;
    lonLatHeight << 115.7804388970, 29.1062965587, 39.4999733213;

    double wgs84_a = 6378137.0;
    double wgs84_invF = 298.257223563;
    Eigen::Vector3d XYZ = toXYZ(lonLatHeight, wgs84_a, wgs84_invF);
    std::cout.precision(10);
    std::cout << std::fixed;

    std::cout << XYZ.transpose() << std::endl;
    return 0;
}

int main2(int argc, char* argv[])
{
    std::cout << RotationOfXYZ2ENU(115.7804388970, 29.1062965587) << std::endl;
    std::cout << RotationOfXYZ2ENU(0, 0) << std::endl;
    std::cout << RotationOfXYZ2ENU(0, 90) << std::endl;
    return 0;
}
int main(int argc, char* argv[])
{
    main0(argc, argv);
    main1(argc, argv);
    main2(argc, argv);
    return 0;
}