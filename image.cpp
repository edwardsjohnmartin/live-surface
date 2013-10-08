#include "./image.h"

#include <iostream>
#include <assert.h>
#include <algorithm>

#include "./texture.h"

using namespace std;
// using namespace cimg_library;

typedef Magick::PixelPacket PixelPacket;
typedef Magick::Pixels Pixels;
// typedef CImg<unsigned char> CImage;

const double Image::MAX = std::numeric_limits<double>::max();

double gradient_max(const Magick::Image& gradient) {
  double m = 0;
  for (int x = 0; x < gradient.columns(); ++x) {
    for (int y = 0; y < gradient.rows(); ++y) {
      m = max(m, (double) gradient.pixelColor(x, y).redQuantum());
    }
  }
  return m;
}

Image::Image(const std::string& fn) : _fn(fn), _w(0), _h(0), _n(0) {
  // _image = CImage((_fn + ".png").c_str());
  // _laplace = CImage((_fn + "-laplace.png").c_str());
  // _gradient = CImage((_fn + "-grad-mag.png").c_str());
  _image = CImage();
  _image.read((_fn + ".png"));
  _laplace.read((_fn + "-laplace.png"));
  _gradient.read((_fn + "-grad-mag.png"));

  // _w = _image.width();
  // _h = _image.height();
  _w = _image.columns();
  _h = _image.rows();
  _n = _w*_h;

  // _gradient_max = _gradient.max();
  _gradient_max = gradient_max(_gradient);

  Init();
}

Image::Image(const string& fn, const CImage& laplace, const CImage& gradient)
    // : _fn(fn), _image((_fn + ".png").c_str()), _laplace(laplace), _gradient(gradient),
    : _fn(fn), _laplace(laplace), _gradient(gradient),
      // _w(laplace.width()), _h(laplace.height()) {
      _w(laplace.columns()), _h(laplace.rows()) {

  _image.read((_fn + ".png"));

  _n = _w*_h;
  // _image = CImg<unsigned char>((_fn + ".png").c_str());
  // _laplace = laplace;
  // _gradient = gradient;
  // _gradient_max = _gradient.max();
  _gradient_max = gradient_max(_gradient);
  
  Init();
}

Image::~Image() {
}

void Image::Init() {
  _wz = 0;
  _wg = 10;

  for (int i = 0; i < 2; ++i) {
    _cost[i].resize(_n, 1);
  }
  for (int x = 0; x < _w; ++x) {
    for (int y = 0; y < _h; ++y) {
      const int idx = Index(x, y);
      // int nbr[2] = { Index(x+1, y), Index(x, y+1) };
      int nbr[2][2] = { {x+1, y}, {x, y+1} };
      for (int i = 0; i < 2; ++i) {
        if (Index(nbr[i][0], nbr[i][1]) < _n) {
          // _cost[i][idx] = _wz * (300 - (*_laplace.data(nbr[i])))/200.0
          //     + _wg * (1 - (*_gradient.data(nbr[i]))/(double)_gradient_max);
          const double l =
              _laplace.pixelColor(nbr[i][0], nbr[i][1]).redQuantum();
          const double g =
              _gradient.pixelColor(nbr[i][0], nbr[i][1]).redQuantum();
          _cost[i][idx] = _wz * (300 - l)/200.0
              + _wg * (1 - g/(double)_gradient_max);
        }
      }
    }
  }
}

// Returns the index of the min cost pixel
std::pair<int, double> Image::MinCost(int x0, int y0, int w, int h) const {
  double minCost = 99999999;
  int minIdx = 0;
  for (int x = x0; x < x0+w; ++x) {
    for (int y = y0; y < y0+h; ++y) {
      const double g =
          _gradient.pixelColor(x, y).redQuantum();
      const double c = _wg * (1 - g/(double)_gradient_max);
      if (c < minCost) {
        minCost = c;
        minIdx = Index(x, y);
      }
    }
  }
  return make_pair(minIdx, minCost);
}

void Image::LoadImages(int texture_id) {
  using namespace std;
  // LoadTexture(_fn + ".jpg", texture_id,
  //             &_w, &_h);

  // InitTexture(texture_id, (unsigned char*) _image, _w, _h, 3, false, false);
  // InitTexture(texture_id, (unsigned char*) _laplace, _w, _h, 1, false, false);
  // InitTexture(texture_id, (unsigned char*) _gradient, _w, _h, 1, false, false);

  // Image _gradient("640x480", "white"); // we'll use the '_gradient' object in this example
  _gradient.modifyImage(); // Ensure that there is only one reference to
  // underlying image; if this is not done, then the
  // image pixels *may* remain unmodified. [???]
  Pixels my_pixel_cache(_gradient); // allocate an image pixel cache associated with _gradient
  PixelPacket* pixels; // 'pixels' is a pointer to a PixelPacket array
  // define the view area that will be accessed via the image pixel cache
  // int start_x = 10, start_y = 20, size_x = 200, size_y = 100; 
  int start_x = 0, start_y = 0, size_x = _w, size_y = _h;
  // return a pointer to the pixels of the defined pixel cache
  pixels = my_pixel_cache.get(start_x, start_y, size_x, size_y);
  // set the color of the first pixel from the pixel cache to black (x=10, y=20 on _gradient)
  // *pixels = Color("black");
  // set to green the pixel 200 from the pixel cache:
  // this pixel is located at x=0, y=1 in the pixel cache (x=10, y=21 on _gradient)
  // *(pixels+200) = Color("green");

  glBindTexture(GL_TEXTURE_2D, texture_id);
  if (QuantumDepth == 8) {
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8, _w, _h, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, (unsigned char*) pixels);
    gluBuild2DMipmaps(
        GL_TEXTURE_2D, GL_RGBA8, _w, _h,
        GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*) pixels);
  } else if (QuantumDepth == 16) {
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA16, _w, _h, 0, GL_RGBA,
        GL_UNSIGNED_SHORT, (void*) pixels);
    gluBuild2DMipmaps(
        GL_TEXTURE_2D, GL_RGBA16, _w, _h,
        GL_RGBA, GL_UNSIGNED_SHORT, (void*) pixels);
  }
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  // InitTexture(texture_id, (unsigned char*) pixels, _w, _h, 4, false, false);

  // now that the operations on my_pixel_cache have been finalized
  // ensure that the pixel cache is transferred back to _gradient
  my_pixel_cache.sync();
}

Image Image::Half() const {
  const int w = _w/2.0+0.5;
  const int h = _h/2.0+0.5;
  // CImage laplace(w, h, 1, 1, 0);
  // CImage gradient(w, h, 1, 1, 0);

  Magick::Geometry geometry(w, h);
  Magick::Color black;
  CImage laplace(geometry, black);
  CImage gradient(geometry, black);

  // x and y are coordinates of original image
  // i and j are coordinates of new image
  for (int x = 0; x < _w; ++x) {
    for (int y = 0; y < _h; ++y) {
      const int i = x/2;
      const int j = y/2;
      // laplace(i, j) = max(laplace(i, j), _laplace(x, y));
      // gradient(i, j) = max(gradient(i, j), _gradient(x, y));
      Magick::Color lcolor(max(laplace.pixelColor(i, j).redQuantum(), _laplace.pixelColor(x, y).redQuantum()), 0, 0);
      // laplace.pixelColor(i, j, lcolor);
      laplace.pixelColor(i, j, Magick::Color("red"));
      Magick::Quantum gq = max(gradient.pixelColor(i, j).redQuantum(), _gradient.pixelColor(x, y).redQuantum());
      Magick::Color gcolor(gq, gq, gq, 1);
      gradient.pixelColor(i, j, gcolor);
    }
  }
  
  return Image(_fn, laplace, gradient);
}

double Image::Cost(int idx0, int idx1) const {
  using namespace std;
  if (abs(idx0-idx1) == 1) {
    return _cost[0][min(idx0, idx1)];
  }
  if (abs(idx0-idx1) == _w) {
    return _cost[1][min(idx0, idx1)];
  }
  cout << "Illegal indices: " << idx0 << " " << idx1 << " (image width = "
       << _w << ")" << endl;
  assert(false);
}

double Image::Cost(int idx) const {
  const Vec2i coords = Coords(idx);
  const double g =
      _gradient.pixelColor(coords[0], coords[1]).redQuantum();
  const double c = _wg * (1 - g/(double)_gradient_max);
  return c;
}

double Image::Cost(int x0, int y0, int x1, int y1) const {
  const int diff = abs(x0-x1) + abs(y0-y1);
  if (diff > 1) {
    return MAX;
  }
  if (diff == 0) {
    return 0;
  }
  if (x0 < x1) {
    return _cost[0][Index(x0, y0)];
  }
  if (x1 < x0) {
    return _cost[0][Index(x1, y1)];
  }
  if (y0 < y1) {
    return _cost[1][Index(x0, y0)];
  }
  return _cost[1][Index(x1, y1)];
}
