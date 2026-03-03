/**
 * @file  numeric.h
 * @brief 数值与线性代数工具：Vec/Mat 类型、旋转、LookAt、clamp 等。
 */

#pragma once
#ifndef INSIGHT_UTIL_NUMERIC_H
#define INSIGHT_UTIL_NUMERIC_H

#include "insight_global.h"

#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>
#include <glog/logging.h>

namespace insight {

#if _WIN32 || _WIN64
#if _WIN64
#define ENV64BIT
#else
#define ENV32BIT
#endif
#elif __GNUC__
#if __x86_64__ || __ppc64__ || _LP64
#define ENV64BIT
#else
#define ENV32BIT
#endif
#endif

using Eigen::Map;
typedef Eigen::NumTraits<double> EigenDoubleTraits;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Quaternion<double> Quaternion;
typedef Eigen::Matrix<double, 3, 3> Mat3;

#if defined(ENV32BIT)
typedef Eigen::Matrix<double, 3, 4, Eigen::DontAlign> Mat34;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vec2;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vec4;
typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign> Vec6;
#else
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
#endif

typedef Eigen::Matrix<double, 4, 4> Mat4;
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> Matu;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMat3;
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> Vecu;
typedef Eigen::MatrixXf Matf;
typedef Eigen::VectorXf Vecf;
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;
typedef Eigen::Matrix<double, Eigen::Dynamic, 9> MatX9;
typedef Eigen::SparseMatrix<double> sMat;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> sRMat;

template <typename T> inline T Square(T x) { return x * x; }

template <typename T> inline T clamp(const T& val, const T& min, const T& max) {
  return std::max(min, std::min(val, max));
}

template <typename T1, typename T2> T2 TruncateCast(const T1 value) {
  return std::min(static_cast<T1>(std::numeric_limits<T2>::max()),
                  std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value));
}

Mat3 CrossProductMatrix(const Vec3& x);
Mat3 RotationAroundX(double angle);
Mat3 RotationAroundY(double angle);
Mat3 RotationAroundZ(double angle);

inline float D2R(const float deg) {
  return deg * 0.0174532925199432954743716805978692718781530857086181640625f;
}
inline double D2R(const double deg) {
  return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}
inline float R2D(const float rad) { return rad * 57.29577951308232286464772187173366546630859375f; }
inline double R2D(const double rad) {
  return rad * 57.29577951308232286464772187173366546630859375;
}

double getRotationMagnitude(const Mat3& R2);
inline double SIGN(double x) { return x < 0.0 ? -1.0 : 1.0; }

template <typename TVec> inline double NormL1(const TVec& x) { return x.array().abs().sum(); }
template <typename TVec> inline double NormL2(const TVec& x) { return x.norm(); }
template <typename TVec> inline double NormLInfinity(const TVec& x) {
  return x.array().abs().maxCoeff();
}
template <typename TVec> inline double DistanceL1(const TVec& x, const TVec& y) {
  return (x - y).array().abs().sum();
}
template <typename TVec> inline double DistanceL2(const TVec& x, const TVec& y) {
  return (x - y).norm();
}
template <typename TVec> inline double DistanceLInfinity(const TVec& x, const TVec& y) {
  return NormLInfinity(x - y);
}

template <typename TMat, typename TVec> double Nullspace(TMat* A, TVec* nullspace) {
  if (A->rows() >= A->cols()) {
    Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
    (*nullspace) = svd.matrixV().col(A->cols() - 1);
    return svd.singularValues()(A->cols() - 1);
  }
  TMat A_extended(A->cols(), A->cols());
  A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
  A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
  return Nullspace(&A_extended, nullspace);
}

template <typename TMat, typename TVec1, typename TVec2>
inline double Nullspace2(TMat* A, TVec1* x1, TVec2* x2) {
  if (A->rows() >= A->cols()) {
    Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
    TMat V = svd.matrixV();
    *x1 = V.col(A->cols() - 1);
    *x2 = V.col(A->cols() - 2);
    return svd.singularValues()(A->cols() - 1);
  }
  TMat A_extended(A->cols(), A->cols());
  A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
  A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
  return Nullspace2(&A_extended, x1, x2);
}

Mat3 LookAt(const Vec3& center, const Vec3& up = Vec3::UnitY());
Mat3 LookAt2(const Vec3& eyePosition3D, const Vec3& center3D = Vec3::Zero(),
             const Vec3& upVector3D = Vec3::UnitY());

#define SUM_OR_DYNAMIC(x, y)                                                                       \
  ((x) == Eigen::Dynamic || (y) == Eigen::Dynamic) ? Eigen::Dynamic : ((x) + (y))

template <typename Derived1, typename Derived2> struct hstack_return {
  typedef typename Derived1::Scalar Scalar;
  enum {
    RowsAtCompileTime = Derived1::RowsAtCompileTime,
    ColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::ColsAtCompileTime, Derived2::ColsAtCompileTime),
    Options = (Derived1::Flags & Eigen::RowMajorBit) ? Eigen::RowMajor : 0,
    MaxRowsAtCompileTime = Derived1::MaxRowsAtCompileTime,
    MaxColsAtCompileTime =
        SUM_OR_DYNAMIC(Derived1::MaxColsAtCompileTime, Derived2::MaxColsAtCompileTime)
  };
  typedef Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime,
                        MaxColsAtCompileTime>
      type;
};

template <typename Derived1, typename Derived2>
typename hstack_return<Derived1, Derived2>::type HStack(const Eigen::MatrixBase<Derived1>& lhs,
                                                        const Eigen::MatrixBase<Derived2>& rhs) {
  typename hstack_return<Derived1, Derived2>::type res;
  res.resize(lhs.rows(), lhs.cols() + rhs.cols());
  res << lhs, rhs;
  return res;
}

template <typename Derived1, typename Derived2> struct vstack_return {
  typedef typename Derived1::Scalar Scalar;
  enum {
    RowsAtCompileTime = SUM_OR_DYNAMIC(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime),
    ColsAtCompileTime = Derived1::ColsAtCompileTime,
    Options = (Derived1::Flags & Eigen::RowMajorBit) ? Eigen::RowMajor : 0,
    MaxRowsAtCompileTime =
        SUM_OR_DYNAMIC(Derived1::MaxRowsAtCompileTime, Derived2::MaxRowsAtCompileTime),
    MaxColsAtCompileTime = Derived1::MaxColsAtCompileTime
  };
  typedef Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime,
                        MaxColsAtCompileTime>
      type;
};

template <typename Derived1, typename Derived2>
typename vstack_return<Derived1, Derived2>::type VStack(const Eigen::MatrixBase<Derived1>& lhs,
                                                        const Eigen::MatrixBase<Derived2>& rhs) {
  typename vstack_return<Derived1, Derived2>::type res;
  res.resize(lhs.rows() + rhs.rows(), lhs.cols());
  res << lhs, rhs;
  return res;
}
#undef SUM_OR_DYNAMIC

template <typename TMat> inline double FrobeniusNorm(const TMat& A) {
  return sqrt(A.array().abs2().sum());
}
template <typename TMat> inline double FrobeniusDistance(const TMat& A, const TMat& B) {
  return FrobeniusNorm(A - B);
}
template <class TMat> double CosinusBetweenMatrices(const TMat& a, const TMat& b) {
  return (a.array() * b.array()).sum() / FrobeniusNorm(a) / FrobeniusNorm(b);
}

template <typename TMat, typename TCols> TMat ExtractColumns(const TMat& A, const TCols& columns) {
  TMat compressed(A.rows(), columns.size());
  for (size_t i = 0; i < static_cast<size_t>(columns.size()); ++i) {
    compressed.col(i) = A.col(columns[i]);
  }
  return compressed;
}

void MeanAndVarianceAlongRows(const Mat& A, Vec* mean_pointer, Vec* variance_pointer);
bool exportMatToTextFile(const Mat& mat, const std::string& filename,
                         const std::string& sPrefix = "A");

inline int is_finite(const double val) {
#ifdef _MSC_VER
  return _finite(val);
#else
  return std::isfinite(val);
#endif
}

template <typename Type, typename DataInputIterator>
void minMaxMeanMedian(DataInputIterator begin, DataInputIterator end, Type& min, Type& max,
                      Type& mean, Type& median) {
  if (std::distance(begin, end) < 1)
    return;
  std::vector<Type> vec_val(begin, end);
  std::sort(vec_val.begin(), vec_val.end());
  min = vec_val[0];
  max = vec_val[vec_val.size() - 1];
  mean =
      std::accumulate(vec_val.begin(), vec_val.end(), Type(0)) / static_cast<Type>(vec_val.size());
  median = vec_val[vec_val.size() / 2];
}

template <typename Type, typename DataInputIterator>
void minMaxMeanMedian(DataInputIterator begin, DataInputIterator end) {
  Type min, max, mean, median;
  minMaxMeanMedian(begin, end, min, max, mean, median);
  std::cout << "\n\t min: " << min << "\n\t mean: " << mean << "\n\t median: " << median
            << "\n\t max: " << max << std::endl;
}

template <typename Type, typename DataInputIterator, typename Stream>
void minMaxMeanMedian(DataInputIterator begin, DataInputIterator end, Stream& s) {
  Type min, max, mean, median;
  minMaxMeanMedian(begin, end, min, max, mean, median);
  s << "\n\t min: " << min << "\n\t mean: " << mean << "\n\t median: " << median
    << "\n\t max: " << max << std::endl;
}

template <typename T>
void SplitRange(const T range_start, const T range_end, const int nb_split,
                std::vector<T>& d_range) {
  const T range_length = range_end - range_start;
  if (range_length < nb_split) {
    d_range.push_back(range_start);
    d_range.push_back(range_end);
  } else {
    const T delta_range = range_length / nb_split;
    d_range.push_back(range_start);
    for (int i = 1; i < nb_split; ++i)
      d_range.push_back(range_start + i * delta_range);
    d_range.push_back(range_end);
  }
}

template <typename T> double Median(const std::vector<T>& elems) {
  CHECK(!elems.empty());
  if (elems.empty())
    return 0;
  const size_t mid_idx = elems.size() / 2;
  std::vector<T> ordered_elems = elems;
  std::nth_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx, ordered_elems.end());
  if (elems.size() % 2 == 0) {
    const T mid_element1 = ordered_elems[mid_idx];
    const T mid_element2 =
        *std::max_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx);
    return (mid_element1 + mid_element2) / 2.0;
  }
  return ordered_elems[mid_idx];
}

inline bool IsNaN(const float x) { return x != x; }
inline bool IsNaN(const double x) { return x != x; }
template <typename Derived> bool IsNaN(const Eigen::MatrixBase<Derived>& x) {
  return !(x.array() == x.array()).all();
}

} // namespace insight

#endif // INSIGHT_UTIL_NUMERIC_H
