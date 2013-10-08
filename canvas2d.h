#ifndef __CANVAS_2D_H__
#define __CANVAS_2D_H__

#include <vector>
#include "./gl2d.h"
#include "./image.h"
// #include "./dijkstra.h"
#include "./dijkstra2.h"
#include "./vec.h"
#include "./snode.h"

class Canvas2D : public GL2D {
 public:
  Canvas2D(const int win_width, const int win_height,
           const std::string& fn);
  ~Canvas2D();

  virtual void Init();
  virtual void Display();
  virtual void Mouse(int button, int state, int x, int y);
  virtual void PassiveMouseMotion(int x, int y);
  virtual void Keyboard(unsigned char key, int x, int y);

  void DrawSimplifyLines(
      // const SNode& node, const Vec2i& pos, const Vec2i& size) const;
      const SNode& node) const;

 private:
  Vec2f Obj2Img(const Vec2f& o) const;
  Vec2f Img2Obj(const Vec2i& i) const;
  Vec2f Img2Obj(const Vec2f& i) const;

 private:
  GLuint* texture_ids;
  int img_width, img_height;
  //          * seed[0]
  //           \
  //            \
  //     *       \
  //   seed[2]    * seed[1]
  std::vector<Vec2f> seeds;
  std::string msg;
  Image image;
  // Dijkstra* dijkstra1;
  // Dijkstra* dijkstra2;
  Dijkstra2* dijkstra1;
  Dijkstra2* dijkstra2;
  Vec2f mouse_cur;
  SNode snode;
};

#endif
