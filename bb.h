#ifndef _123_BOUNDING_BOX_H_
#define _123_BOUNDING_BOX_H_

#include <algorithm>

#include "./vec.h"

using namespace std;

template <typename NumType, int D>
class BoundingBox {
 public:
  typedef Vec<NumType, D> Point;

 public:
  BoundingBox() : _initialized(false) {}
  BoundingBox(const Point& a, const Point& b) : _initialized(false) {
    (*this)(a);
    (*this)(b);
  }

  Point center() const {
    return Point((_min+_max)/2);
  }

  Point size() const { return _max - _min; }
  Point min() const { return _min; }
  Point max() const { return _max; }
  NumType max_size() const {
    const Point s = size();
    NumType m = s[0];
    for (int i = 1; i < D; ++i) {
      m = (s[i]>m)?s[i]:m;
    }
    return m;
  }

  bool in_open(const Point& p) const {
    for (int i = 0; i < D; ++i) {
      if (p[i] <= _min[i] || p[i] >= _max[i]) return false;
    }
    return true;
  }

  bool in_closed(const Point& p) const {
    for (int i = 0; i < D; ++i) {
      if (p[i] < _min[i] || p[i] > _max[i]) return false;
    }
    return true;
  }

  void operator()(const Point& v) {
    if (_initialized) {
      _min = Point::min(_min, v);
      _max = Point::max(_max, v);
    } else {
      _min = v;
      _max = v;
      _initialized = true;
    }
  }

  void operator+=(const BoundingBox& rhs) {
    if (_initialized) {
      (*this)(rhs._min);
      (*this)(rhs._max);
    } else {
      *this = rhs;
    }
  }

  // Resize by f in each dimension
  BoundingBox operator*(const NumType f) const {
    if (_initialized) {
      const Point s = size() * f;
      const Point c = center();
      Point mi = c - s/2;
      Point ma = c + s/2;
      BoundingBox ret;
      ret(mi);
      ret(ma);
      return ret;
    } else {
      return *this;
    }
  }

 private:
  bool _initialized;
  Point _min;
  Point _max;
};

template <typename NumType, int D>
inline ostream & operator<<(ostream & out, const BoundingBox<NumType, D>& b) {
  out << "[box min=" << b.min()
      << " max=" << b.max()
      << " center=" << b.center() << "]";
  return out;
}

typedef BoundingBox<float, 3> BoundingBox3f;

#endif
