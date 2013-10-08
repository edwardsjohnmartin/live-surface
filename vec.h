#ifndef _VEC_H_
#define _VEC_H_

#include <cmath>
#include <cstring>

#include <algorithm>
#include <iostream>
#include <stdexcept>

// typedef unsigned short ushort;
typedef unsigned char myushort;

template <class NumType, myushort NumDims>
struct Vec {
  static const myushort Dim = NumDims;
  static const myushort D = NumDims;
  NumType x[NumDims];

  Vec(NumType a = 0, NumType b = 0, NumType c = 0, NumType d = 0) {
    NumType v[] = { a, b, c, d };
    memcpy(x, v, std::min(NumDims, static_cast<myushort>(4)) * sizeof(a));
  }

  Vec(const Vec<NumType, NumDims-1>& v, NumType a = 0) {
    for (int i = 0; i < NumDims-1; ++i) {
      x[i] = v[i];
    }
    x[NumDims-1] = a;
  }

  explicit Vec(NumType* values) {
    memcpy(x, values, NumDims * sizeof(values[0]));
  }

  static Vec<NumType, NumDims> zero() {
    Vec<NumType, NumDims> z;
    memset(z.x, 0, sizeof(NumType)*NumDims);
    return z;
  }
  void copy(const NumType* y) {
    memcpy(x, y, sizeof(NumType)*NumDims);
  }
  void copy(const Vec<NumType, NumDims>& v) {
    memcpy(x, v.x, sizeof(NumType)*NumDims);
  }
  NumType norm2() const {
    if (0 == (NumType)0.1) {
      // Can't call norm2() with an integer type
      throw std::logic_error("can't get norm2 of integer type");
    }
    return (*this)*(*this);
  }
  NumType norm() const {
    if (0 == (NumType)0.1) {
      // If integer type, use double's norm
      return (NumType) (Vec<double, D>(*this).norm()+0.5);
    }
    return sqrt(norm2());
  }
  Vec<NumType, NumDims> unit() const {
    return (*this)/norm();
  }
  bool operator==(NumType a) const {
    bool eq = true;
    for (int i = 0; i < NumDims && (eq &= (x[i] == a)); ++i);
    return eq;
  }
  bool operator==(const Vec<NumType, NumDims>& rhs) const {
    bool eq = true;
    for (int i = 0; i < NumDims && (eq &= (x[i] == rhs.x[i])); i++);
    return eq;
  }
  bool operator!=(const Vec<NumType, NumDims>& rhs) const {
    return !(*this == rhs);
  }

  Vec<NumType, NumDims> prod(const Vec<NumType, NumDims>& rhs) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result[i] = x[i]*rhs.x[i];
    return result;
  }
  NumType operator*(const Vec<NumType, NumDims>& rhs) const {
    NumType result = NumType(0);
    for (int i = 0; i < NumDims; i++) result += x[i]*rhs.x[i];
    return result;
  }
  // This implementation fails on integer types
  // Vec<NumType, NumDims> operator/(NumType a) const {
  //   Vec<NumType, NumDims> result;
  //   const NumType inv = 1.0 / a;
  //   for (int i = 0; i < NumDims; i++) result.x[i] = x[i] * inv;
  //   return result;
  // }
  Vec<NumType, NumDims> operator/(NumType a) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i] / a;
    return result;
  }
  Vec<NumType, NumDims> operator*(NumType a) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i]*a;
    return result;
  }
  Vec<NumType, NumDims> operator+(const Vec<NumType, NumDims>& rhs) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i]+rhs.x[i];
    return result;
  }
  Vec<NumType, NumDims> operator+(NumType a) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i]+a;
    return result;
  }
  Vec<NumType, NumDims> operator-(const Vec<NumType, NumDims>& rhs) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i]-rhs.x[i];
    return result;
  }
  Vec<NumType, NumDims> operator-(NumType a) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = x[i]-a;
    return result;
  }
  Vec<NumType, NumDims> operator-() const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) result.x[i] = -x[i];
    return result;
  }
  Vec<NumType, NumDims>& operator+=(const Vec<NumType, NumDims>& rhs) {
    return *this=(*this)+rhs;
  }
  Vec<NumType, NumDims>& operator+=(const NumType rhs) {
    return *this=(*this)+rhs;
  }
  Vec<NumType, NumDims>& operator-=(const Vec<NumType, NumDims>& rhs) {
    return *this=(*this)-rhs;
  }
  Vec<NumType, NumDims>& operator-=(const NumType rhs) {
    return *this=(*this)-rhs;
  }
  Vec<NumType, NumDims>& operator*=(NumType a) {
    return *this=(*this)*a;
  }
  Vec<NumType, NumDims>& operator/=(NumType a) {
    return *this=(*this)/a;
  }
  Vec<NumType, NumDims> operator/(const Vec<NumType, NumDims> rhs) const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; ++i)
      result[i] = x[i] / rhs.x[i];
    return result;
  }

  Vec<NumType, NumDims> operator^(const Vec<NumType, NumDims>& rhs) const {
    Vec<NumType, NumDims> result;
    if (NumDims >= 3) {
      result.x[0] = x[1]*rhs.x[2]-x[2]*rhs.x[1];
      result.x[1] = x[2]*rhs.x[0]-x[0]*rhs.x[2];
      result.x[2] = x[0]*rhs.x[1]-x[1]*rhs.x[0];
    }
    return result;
  }

  Vec<NumType, NumDims>& operator^=(const Vec<NumType, NumDims>& rhs) {
    return *this=(*this)^rhs;
  }

  bool operator<(NumType rhs) const {
    bool lt = true;
    for (int i = 0; i < NumDims && (lt = (x[i] < rhs)); i++);
    return lt;
  }

  bool operator<(const Vec<NumType, NumDims>& rhs) const {
    // bool lt = true;
    // for (int i = 0; i < NumDims && (lt = (x[i] < rhs[i])); i++);
    for (int i = 0; i < NumDims; ++i) {
      if (x[i] < rhs[i]) return true;
      if (x[i] > rhs[i]) return false;
    }
    // return lt;
    return false;
  }

  inline NumType &operator[](int _n) {
    return x[_n];
  }

  inline NumType operator[](int _n) const {
    return x[_n];
  }

  operator const NumType*() const {
    return &x[0];
  }

  operator const Vec<float, NumDims>() const {
    Vec<float, NumDims> v;
    for (int i = 0; i < NumDims; ++i) v[i] = x[i];
    return v;
  }

  operator const Vec<double, NumDims>() const {
    Vec<double, NumDims> v;
    for (int i = 0; i < NumDims; ++i) v[i] = x[i];
    return v;
  }

  operator const Vec<int, NumDims>() const {
    Vec<int, NumDims> v;
    for (int i = 0; i < NumDims; ++i) v[i] = static_cast<int>(x[i]);
    return v;
  }

  static Vec<NumType, NumDims> min(
      const Vec<NumType, NumDims>& a, const Vec<NumType, NumDims>& b) {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) {
      result.x[i] = (a.x[i] < b.x[i] ? a.x[i] : b.x[i]);
    }
    return result;
  }

  static Vec<NumType, NumDims> max(
      const Vec<NumType, NumDims>& a, const Vec<NumType, NumDims>& b) {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; ++i) {
      result.x[i]=(a.x[i]>b.x[i]?a.x[i]:b.x[i]);
    }
    return result;
  }

  Vec<NumType, NumDims> abs() const {
    Vec<NumType, NumDims> result;
    for (int i = 0; i < NumDims; i++) {
      result.x[i]=(x[i]>0?x[i]:-x[i]);
    }
    return result;
  }

  NumType min() const {
    NumType result = x[0];
    for (int i = 0; i < NumDims; i++) {
      result= (result < x[i] ? result : x[i]);
    }
    return result;
  }


  NumType max() const {
    NumType result = x[0];
    for (int i = 0; i < NumDims; i++) {
      result = (result>x[i]?result:x[i]);
    }
    return result;
  }

  template <class NType, myushort NDims>
  friend std::ostream& operator<<(
      std::ostream& out, const Vec<NType, NDims>& v);
  template <class NType, myushort NDims>
  friend Vec<NType, NDims> operator*(NType a, const Vec<NType, NDims>& rhs);
};

template<class NType, myushort NDims>
Vec<NType, NDims> operator*(NType a, const Vec<NType, NDims>& rhs) {
  return rhs*a;
}

template<class NType, myushort NDims>
std::ostream& operator<<(std::ostream& out, const Vec<NType, NDims>& v) {
  out << "(";
  for (int i = 0; i < NDims; i++) {
    out << v.x[i];
    if (i < NDims-1) out << ",";
  }
  out << ")";
  return out;
}

typedef Vec<int, 2> Vec2i;
typedef Vec<float, 2> Vec2f;
typedef Vec<double, 2> Vec2d;
typedef Vec<unsigned char, 2> Vec2b;

typedef Vec<int, 3> Vec3i;
typedef Vec<float, 3> Vec3f;
typedef Vec<double, 3> Vec3d;
typedef Vec<unsigned char, 3> Vec3b;

typedef Vec<int, 4> Vec4i;
typedef Vec<float, 4> Vec4f;
typedef Vec<double, 4> Vec4d;
typedef Vec<unsigned char, 4> Vec4b;

#endif
