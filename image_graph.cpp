#include "image_graph.h"

#include <algorithm>

ImageGraph::ImageGraph(const Image& image)
    : _image(image) {
  _graph.resize(3*(int)(image.Width()*image.Height()/4.0 + 0.75) + 1);
  // _graph.resize(image.Width()*image.Height());
  _img2graph.resize(image.Width()*image.Height());
  _graph2img.resize(_graph.size());
  int idx = 0;

  // graph index
  int gi = 0;
  // top half
  for (int i = 0; i < _image.Width(); ++i) {
    for (int j = 0; j < _image.Height()/2; ++j) {
      const int k = _image.Index(i, j);
      _img2graph[k] = gi;
      _graph2img[gi] = k;
      ++gi;
    }
  }
  // bottom-left
  for (int i = 0; i < _image.Width()/2; ++i) {
    for (int j = _image.Height()/2; j < _image.Height(); ++j) {
      const int k = _image.Index(i, j);
      _img2graph[k] = gi;
      _graph2img[gi] = k;
      ++gi;
    }
  }
  // bottom-right
  for (int i = _image.Width()/2; i < _image.Width(); ++i) {
    for (int j = _image.Height()/2; j < _image.Height(); ++j) {
      const int k = _image.Index(i, j);
      _img2graph[k] = gi;
    }
  }
  _graph2img[gi] = _image.Index(3*_image.Width()/4, 3*_image.Height()/4);

  // Artificial pixel
  const int w = _image.Width()/2;
  const int h = _image.Height()/2;
  const double collapsedCost = _image.MinCost(w, h, w, h).second * std::min(w/2, h/2);

  // Fill in connectivity (edges w/neighbors, etc)
  typedef std::vector<int>::const_iterator Iter;
  for (int i = 0; i < _graph2img.size(); ++i) {
    std::vector<int> imgNbrs;
    const int graphIdx = i;
    const int imgIdx = _graph2img[i];
    if (i < _graph2img.size()-1) {
      _image.Neighbors(imgIdx, back_inserter(imgNbrs));
    } else {
      // Bottom-right pixel.  Make neighbors with bordering pixels.
      for (int x = _image.Width()/2-1; x < _image.Width(); ++x) {
        const int y = _image.Height()/2-1;
        imgNbrs.push_back(_image.Index(x, y));
      }
      for (int y = _image.Height()/2-1; y < _image.Height(); ++y) {
        const int x = _image.Width()/2-1;
        imgNbrs.push_back(_image.Index(x, y));
      }
    }
    std::vector<int> graphNbrs;
    for (int i = 0; i < imgNbrs.size(); ++i) {
      graphNbrs.push_back(_img2graph[imgNbrs[i]]);
    }
    std::vector<double> costs(graphNbrs.size());
    for (int j = 0; j < graphNbrs.size(); ++j) {
      double c;
      const int nbrIdx = imgNbrs[j];
      if (abs(imgIdx-nbrIdx) == 1 || abs(imgIdx-nbrIdx) == _image.Width()) {
        // Actually neighbors.
        c = _image.Cost(imgIdx, nbrIdx);
      } else {
        c = collapsedCost;
      }
      costs[j] = c;
    }
    // _graph[graphIdx] = GNode(graphNbrs, costs);
    _graph[graphIdx] = GNode(graphNbrs, _image.Cost(imgIdx));
  }
}

// imgIdx - image index
// gIdx - graph index
// void ImageGraph::InitNode(const int imgIdx, const int gIdx) {
  // std::vector<int> nbrs;
  // _image.Neighbors(imgIdx, back_inserter(nbrs));
  // std::vector<double> costs(nbrs.size());
  // for (int j = 0; j < nbrs.size(); ++j) {
  //   costs[j] = _image.Cost(imgIdx, nbrs[j]);
  // }
  // _graph[gIdx] = GNode(nbrs, costs);
// }

int ImageGraph::ImageIndex(const int x, const int y) const {
  return _image.Index(x, y);
}

int ImageGraph::GraphIndex(const int x, const int y) const {
  return _img2graph[ImageIndex(x, y)];
}

// Index given in graph coordinates
Vec2i ImageGraph::Coords(int index) const {
  return _image.Coords(_graph2img[index]);
}

int ImageGraph::Size() const {
  return _graph.size();
}

double ImageGraph::Cost(int x0, int y0, int x1, int y1) const {
  // return _image.Cost(x0, y0, x1, y1);
  const int img0 = _img2graph[_image.Index(x0, y0)];
  const int img1 = _img2graph[_image.Index(x1, y1)];
  // return Cost(Index(x0, y0), Index(x1, y1));
  return Cost(img0, img1);
}

// Indices are in graph coordinates
double ImageGraph::Cost(int idx0, int idx1) const {
  // return _image.Cost(idx0, idx1);
  // return _graph[idx0].Cost(idx1);
  return _graph[idx0].Cost(_graph[idx1]);
}
