#ifndef __IMAGE_GRAPH_H__
#define __IMAGE_GRAPH_H__

#include "./image.h"
#include "./gnode.h"

class ImageGraph {
 public:
  ImageGraph(const Image& image);

  // imgIdx - image index
  // gIdx - graph index
  // void InitNode(const int imgIdx, const int gIdx);
  int ImageIndex(const int x, const int y) const;
  int GraphIndex(const int x, const int y) const;

  // Index given in graph coordinates
  Vec2i Coords(int index) const;
  int Size() const;

  template <typename Out_iter>
  void Neighbors(int idx, Out_iter out) const {
    for (int i = 0; i < _graph[idx].Degree(0); ++i) {
      *out++ = _graph[idx].Nbr(0, i);
    }
    // _image.Neighbors(idx, out);
  }

  double Cost(int x0, int y0, int x1, int y1) const;

  // Indices are in graph coordinates
  double Cost(int idx0, int idx1) const;

 private:
  const Image& _image;
  std::vector<GNode> _graph;
  std::vector<int> _img2graph;
  std::vector<int> _graph2img;
  
};


#endif
