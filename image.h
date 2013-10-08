#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <limits>
#include <vector>
#include <string>

// #include "./Cimg.h"
#include <Magick++.h>
#include "./vec.h"

class Image {
 private:
  static const double MAX;
  // typedef cimg_library::CImg<unsigned char> CImage;
  typedef Magick::Image CImage;

 public:
  Image(const std::string& fn);
  Image(const std::string& fn, const CImage& laplace, const CImage& gradient);
  ~Image();

  void Init();

  void LoadImages(int texture_id);

  double Cost(int x0, int y0, int x1, int y1) const;
  double Cost(int idx0, int idx1) const;
  double Cost(int idx) const;
  // Returns the index of the min cost pixel
  std::pair<int, double> MinCost(int x0, int y0, int w, int h) const;
  int Index(int x, int y) const {
    return y * _w + x;
  }
  // std::pair<int, int> Coords(int idx) const {
  //   return std::make_pair(idx % _w, idx / _w);
  // }
  Vec2i Coords(int idx) const {
    return Vec2i(idx % _w, idx / _w);
  }
  inline int Neighbor(int idx, int dx, int dy) const {
    return idx + dx + dy*_w;
  }
  template <typename Out_iter>
  void Neighbors(int idx, Out_iter out) const {
    if (idx % _w != 0) *out++ = Neighbor(idx, -1, 0);
    if (idx % _w != _w-1) *out++ = Neighbor(idx, 1, 0);
    if (idx >= _w) *out++ = Neighbor(idx, 0, -1);
    if (idx < Size() - _w) *out++ = Neighbor(idx, 0, 1);
  }

  int Width() const { return _w; }
  int Height() const { return _h; }
  int Size() const {
    return _w * _h;
  }

  // Cut this image in half in each dimension
  Image Half() const;

 private:
  std::string _fn;
  CImage _image;
  CImage _laplace;
  CImage _gradient;
  double _gradient_max;
  double _wz;
  double _wg;
  int _w;
  int _h;
  int _n;
  // cost[0][i] (resp. cost[1][i]) is the cost from pixel i to the
  // neighbor pixel in the positive x (resp. y) direction.
  std::vector<double> _cost[2];
};

#endif
