#ifndef __GL_3D_H__
#define __GL_3D_H__

#include <string>

#include "./bb.h"
#include "./common.h"
#include "./material.h"
#include "./scene.h"

class GL3D : public Scene {
 public:
  typedef BoundingBox<GLfloat, 2> BB;

  enum Justify { kLeftJustify, kRightJustify,
                 kTopJustify, kBottomJustify,
                 kCenterJustify };

 public:
  GL3D(const int win_width, const int win_height) {//, const Vec2f& world_min, const Vec2f& world_max) {
    Reshape(win_width, win_height);//, world_min, world_max);
  }
  virtual ~GL3D() {}

  virtual void Reshape(const int win_width, const int win_height);
                       // const Vec2f& world_min, const Vec2f& world_max);

  virtual int ProcessArgs(int argc, char** argv) { return 0; }
  virtual void Init() {}
  virtual void Mouse(int button, int state, int x, int y) {}
  virtual void MouseMotion(int x, int y) {}
  virtual void PassiveMouseMotion(int x, int y) {}
  virtual void Keyboard(unsigned char key, int x, int y) {}
  virtual void Special(unsigned char key, int x, int y) {}
  virtual void Display() {}

  static void InitColors();
  static Vec3f RandomColor();
  // Returned color may not be close to avoid
  static Vec3f RandomColor(int seed, const Vec3f& avoid);

  Vec3f Win2Obj(const Vec3f& p) const;
  Vec3f Obj2Win(const Vec3f& obj) const;

  // p is in window coordinates.
  void BitmapString(const string& s, const Vec2i& p,
                          void* font = GLUT_BITMAP_8_BY_13) const;

  // p is in object coordinates
  void BitmapString(const string& s, const Vec3f& p,
                    void* font = GLUT_BITMAP_8_BY_13) const;

// obj is in world coordinates.
// xoff, yoff are in window coordinates
  void BitmapString(const string& s, const Vec3f& obj,
                  int xoff, int yoff,
                    void* font = GLUT_BITMAP_8_BY_13) const;

  void BitmapString(int value, const Vec3f& obj,
                  int xoff = 1, int yoff = 1,
                    void* font = GLUT_BITMAP_8_BY_13) const;

  void BitmapString(const string& s, const Vec3f& obj,
                    Justify hjustify, Justify vjustify) const;

  void BitmapString(int value, const Vec3f& obj,
                    Justify hjustify, Justify vjustify) const;

  void BitmapString(const string& s) const;

// size = edge length
  void glCube(const Vec3f& obj, int size) const;
  void SetMaterial(const Material& m) const;
  void SetDiffuseAmbient(const float r, const float g, const float b) const;

 protected:
  int window_width;
  int window_height;
  // GLfloat fwindow_width;
  // GLfloat fwindow_height;
  float window_aspect;
  // BB win_obj;
  static std::vector<Vec3f> colors;
};

#endif
