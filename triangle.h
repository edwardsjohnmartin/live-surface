#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

#include <vector>
#include <stdexcept>
#include <utility>
#include <iostream>
#include <algorithm>
#include <map>

#include "./edge.h"

class Triangle {
 public:
  typedef std::vector<int>::iterator iterator;
  typedef std::vector<int>::const_iterator const_iterator;

 public:
  Triangle() {}
  Triangle(int v0, int v1, int v2) {
    _i[0] = v0;
    _i[1] = v1;
    _i[2] = v2;
  }
  Triangle(int v[]) {
    _i[0] = v[0];
    _i[1] = v[1];
    _i[2] = v[2];
  }

  Triangle inverted() const {
    return Triangle(_i[2], _i[1], _i[0]);
  }

  // Given a local vertex index, return the
  // global vertex index.
  int l2g(int i) const { return _i[i]; }

  // Given a global vertex index, return the
  // local vertex index.  Returns -1 if vi isn't
  // a vertex.
  int g2l(int vi) const {
    for (int i = 0; i < 3; ++i) {
      if (_i[i] == vi) {
        return i;
      }
    }
    return -1;
  }

  int next(int vi) const {
    return l2g((g2l(vi)+1)%3);
  }

  int prev(int vi) const {
    return l2g((g2l(vi)+2)%3);
  }

  size_t size() const { return 3; }

  template <typename Out_iter>
  void get_edges(Out_iter out) const {
    for (int i = 0; i < 3; ++i) {
      *out++ = Edge(_i[i], _i[(i+1)%3]);
    }
  }

  // Given a global vertex index, return the
  // remaining two global vertex indices in
  // their original order.
  Edge opposite(int vi) const {
    return Edge(prev(vi), next(vi));
  }

  bool has_edge(int vi, int vj) const {
    int vii = -1;
    for (int i = 0; vii == -1 && i < 3; ++i) {
      if (_i[i] == vi) vii = i;
    }
    if (vii == -1) return false;

    int vji = -1;
    for (int i = 0; vji == -1 && i < 3; ++i) {
      if (_i[i] == vj) vji = i;
    }
    return (vji != -1);
  }

  bool has_edge(const Edge& e) const {
    return has_edge(e[0], e[1]);
  }

  // Return the point at the given local vertex index.
  // Point_3 operator[](size_t i) const { return (*_v)[_i[i]]; }
  int operator[](size_t i) const { return _i[i%3]; }

  bool operator==(const Triangle& rhs) const {
    for (int i = 0; i < 3; ++i) {
      int li = -1;
      for (int j = 0; li == -1 && j < 3; ++j) {
        if (_i[i] == rhs[j]) li = j;
      }
      if (li == -1) return false;
    }
    return true;
  }

  bool operator!=(const Triangle& rhs) const {
    return !(*this == rhs);
  }

  bool operator<(const Triangle& t) const { 
    int sorted[3] = { _i[0], _i[1], _i[2] };
    int rsorted[3] = { t._i[0], t._i[1], t._i[2] };
    std::sort(sorted, sorted+3);
    std::sort(rsorted, rsorted+3);
    for (int i = 0; i < 3; ++i) {
      if (sorted[i] != rsorted[i]) {
        return sorted[i] < rsorted[i];
      }
    }
    return false;
  }

 private:
  int _i[3];
};

inline std::ostream& operator<<(std::ostream& out, const Triangle& t) {
  out << t[0] << " " << t[1] << " " << t[2];
  return out;
}

inline std::istream& operator>>(std::istream& in, Triangle& t) {
  int i, j, k;
  in >> i >> j >> k;
  t = Triangle(i, j, k);
  return in;
}

template <typename Iter>
std::vector<std::vector<Triangle> > build_v2t(Iter begin, Iter end)
{
  int m = -1;
  for (Iter it = begin; it != end; ++it) {
    const Triangle& t = *it;
    for (int i = 0; i < 3; ++i) {
      m = std::max(m, t[i]);
    }
  }
  std::vector<std::vector<Triangle> > ret(m+1);
  for (Iter it = begin; it != end; ++it) {
    const Triangle& t = *it;
    for (int i = 0; i < 3; ++i) {
      ret[t[i]].push_back(t);
    }
  }
  return ret;
}

template <typename Iter>
std::map<Edge, std::vector<Triangle> > build_e2t(Iter begin, Iter end)
{
  std::map<Edge, std::vector<Triangle> > ret;
  for (Iter it = begin; it != end; ++it) {
    const Triangle& t = *it;
    for (int i = 0; i < 3; ++i) {
      ret[t.opposite(t[i])].push_back(t);
    }
  }
  return ret;
}

#endif
