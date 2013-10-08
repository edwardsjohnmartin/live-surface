#ifndef __GL_2D_H__
#define __GL_2D_H__

#include <string>

#include "./bb.h"
#include "./scene.h"
#include "./common.h"

class GL2D : public Scene {
 public:
  typedef BoundingBox<GLfloat, 2> BB;

  enum Justify { kLeftJustify, kRightJustify,
                 kTopJustify, kBottomJustify,
                 kCenterJustify };

 public:
  // GL2D(const Vec2i& win, const Vec2f& world_min, const Vec2f& world_max) {
  //   Reshape(win);//, world_min, world_max);
  GL2D(const int win_width, const int win_height) {
    Reshape(win_width, win_height);//, world_min, world_max);
    // window_width = win[0];
    // window_height = win[1];
    // fwindow_width = static_cast<GLfloat>(window_width);
    // fwindow_height = static_cast<GLfloat>(window_height);
    // window_aspect = fwindow_width / fwindow_height;
    // win_obj = BB(world_min, world_max);
  }
  virtual ~GL2D() {}

  virtual void Reshape(const int win_width, const int win_height);
  virtual void Reshape(const int win_width, const int win_height,
                       const BB& viewport);
  // virtual void Reshape(const Vec2i& w,
  //                      const Vec2f& world_min, const Vec2f& world_max);

  virtual void ProcessArgs(int argc, char** argv) {}
  virtual void Init();
  virtual void Mouse(int button, int state, int x, int y) {}
  virtual void MouseMotion(int x, int y) {}
  virtual void PassiveMouseMotion(int x, int y) {}
  virtual void Keyboard(unsigned char key, int x, int y) {}
  virtual void Special(int key, int x, int y) {}
  virtual void Display() {}

  Vec2f Win2Obj(const Vec2f& w) const;
  float Win2Obj(const float w) const;
  Vec2f Obj2Win(const Vec2f& v) const;
  Vec2f Obj2Win(const float x, const float y) const;

  void BitmapString(const string& s, const Vec2f& p,
                    void* font = GLUT_BITMAP_8_BY_13) const;
  void BitmapString(const string& s, const Vec2f& obj,
                    int xoff = 1, int yoff = 1,
                    void* font = GLUT_BITMAP_8_BY_13) const;
  void BitmapString(int value, const Vec2f& obj,
                    int xoff = 1, int yoff = 1,
                    void* font = GLUT_BITMAP_8_BY_13) const;
  void BitmapString(const string& s, const Vec2f& obj,
                    Justify hjustify, Justify vjustify) const;
  void BitmapString(int value, const Vec2f& obj,
                    Justify hjustify, Justify vjustify) const;
  void BitmapString(const string& s) const;

  Vec3f RandomColor() const;
  Vec3f RandomColor(const Vec3f& avoid) const;
  Vec3f RandomColor(int seed, const Vec3f& avoid) const;
  void SetColor(int seed, const Vec3f& avoid) const;

  // Set the viewport in object coordinates
  void SetViewObj(const BB& vo) { view_obj = vo; }

 protected:
  int window_width;
  int window_height;
  GLfloat fwindow_width;
  GLfloat fwindow_height;
  float window_aspect;
  BB win_obj;
  BB view_obj;
};

#endif
