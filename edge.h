#ifndef __EDGE_H__
#define __EDGE_H__

#include <vector>
#include <algorithm>
#include <iostream>

class Edge {
 public:
  typedef std::vector<int>::iterator iterator;
  typedef std::vector<int>::const_iterator const_iterator;

 public:
  Edge() : _i(2), _sorted(2) {}
  Edge(int i, int j) : _i(2), _sorted(2) {
    _i[0] = i;
    _i[1] = j;
    _sorted[0] = i;
    _sorted[1] = j;
    if (i > j) {
      std::swap(_sorted[0], _sorted[1]);
    }
  }

  size_t size() const { return 2; }

  iterator begin() { return _i.begin(); }
  const_iterator begin() const { return _i.begin(); }
  iterator end() { return _i.end(); }
  const_iterator end() const { return _i.end(); }

  int operator[](int i) const {
    return _i[i];
  }

  int at(int i) const {
    return _i[i];
  }

  int sl2g(int i) const {
    return _sorted[i];
  }

  int g2l(int vi) const {
    if (_i[0] == vi) return 0;
    if (_i[1] == vi) return 1;
    return -1;
  }

  bool operator<(const Edge& e) const {
    if (_sorted[0] == e._sorted[0]) {
      return _sorted[1] < e._sorted[1];
    }
    return _sorted[0] < e._sorted[0];
  }

  bool operator==(const Edge& e) const {
    return (_sorted[0] == e._sorted[0] &&
            _sorted[1] == e._sorted[1]);
  }

  bool operator!=(const Edge& e) const {
    return !(*this == e);
  }

 private:
  std::vector<int> _i;
  std::vector<int> _sorted;
};

inline std::ostream& operator<<(std::ostream& out, const Edge& e) {
  out << e[0] << " " << e[1];
  return out;
}

inline std::istream& operator>>(std::istream& in, Edge& e) {
  int a, b;
  in >> a >> b;
  e = Edge(a, b);
  return in;
}

// Useful for the connect() algorithm.
inline int edge_at(const Edge& e, int i) {
  return e[i];
}

#endif
