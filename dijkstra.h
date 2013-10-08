#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__

#include <assert.h>
#include <limits>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "./image.h"

class Dijkstra {
 private:
  static const double MAX;

 public:
  Dijkstra(const Image& image);

  void Run(const int seed);
  void Run(const int seedx, const int seedy);

  int Parent(const int x, const int y) const {
    return _parent[_image.Index(x, y)];
  }

  template <typename Out_iter>
  void Path(const int curx, const int cury, Out_iter out) const {
    // std::cout << "Starting path from " << curx << ", " << cury << std::endl;
    int cur = _image.Index(curx, cury);
    do {
      if (cur == -1) {
        std::cerr << "cur = -1" << std::endl;
        throw std::logic_error("cur = -1");
      }
      // std::cout << cur << std::endl;
      *out++ = _image.Coords(cur);
      cur = _parent[cur];
    } while (_parent[cur] != cur);
  }

 private:
  const Image& _image;
  std::vector<int> _parent;
  std::vector<double> _cost;
};

#endif
