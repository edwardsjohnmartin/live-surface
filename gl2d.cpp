#include "./gl2d.h"

#include <sstream>
#include <string>

using namespace std;

void GL2D::Init() {
}

void GL2D::Reshape(const int win_width, const int win_height,
                   const BB& viewport) {
  window_width = win_width;
  window_height = win_height;
  fwindow_width = static_cast<GLfloat>(window_width);
  fwindow_height = static_cast<GLfloat>(window_height);
  window_aspect = fwindow_width / fwindow_height;
  const Vec2f win_min_obj(-window_aspect, -1);
  const Vec2f win_max_obj(window_aspect, 1);
  win_obj = BB(win_min_obj, win_max_obj);

  // const Vec2f world_min(-window_aspect
  //                       + 2*window_aspect*(viewport.min()[0]/window_width),
  //                       -1 + 2*viewport.min()[1]/window_height);
  // const Vec2f world_max(-window_aspect
  //                       + 2*window_aspect*(viewport.max()[0]/window_width),
  //                       -1 + 2*viewport.max()[1]/window_height);
  // view_obj = BB(world_min, world_max);
  Init();
}

void GL2D::Reshape(const int win_width, const int win_height) {
  Reshape(win_width, win_height, BB(Vec2f(0, 0), Vec2f(win_width, win_height)));
  // window_width = win_width;
  // window_height = win_height;
  // fwindow_width = static_cast<GLfloat>(window_width);
  // fwindow_height = static_cast<GLfloat>(window_height);
  // window_aspect = fwindow_width / fwindow_height;
  // const Vec2f world_min(-window_aspect, -1);
  // const Vec2f world_max(window_aspect, 1);
  // win_obj = BB(world_min, world_max);
  // Init();
}

Vec2f GL2D::Win2Obj(const Vec2f& w) const {
  // const Vec2f& mi = win_obj.min();
  // const Vec2f& ma = win_obj.max();
  // const GLfloat obj_width = (ma[0]-mi[0]);
  // const GLfloat obj_height = (ma[1]-mi[1]);
  // Vec2f winobj(
  //     (w[0] / fwindow_width) * (obj_width) + mi[0],
  //     ((window_height-w[1]) / fwindow_height) * obj_height + mi[1]);

  // const GLfloat view_width = view_obj.size()[0];
  // const GLfloat view_height = view_obj.size()[1];
  // return Vec2f(view_width*(winobj[0]-mi[0])/obj_width + view_obj.min()[0],
  //              view_height*(winobj[1]-mi[1])/obj_height + view_obj.min()[1]);
//  const Vec2f& mi = view_obj.min();
//  const Vec2f& ma = view_obj.max();

  const GLfloat obj_width = win_obj.size()[0];
  const GLfloat obj_height = win_obj.size()[1];
  return Vec2f(
      (w[0] / fwindow_width) * (obj_width) + win_obj.min()[0],
      ((window_height-w[1]) / fwindow_height) * obj_height + win_obj.min()[1]);
}

float GL2D::Win2Obj(const float w) const {
  const GLfloat obj_width = win_obj.size()[0];
  return (w / fwindow_width) * (obj_width);
}

Vec2f GL2D::Obj2Win(const Vec2f& v) const {
  const GLfloat obj_width = win_obj.size()[0];
  const GLfloat obj_height = win_obj.size()[1];
  const Vec2f offset = v - win_obj.min();
  return Vec2f(
      fwindow_width * offset[0]/obj_width,
      fwindow_height * (1.0 - offset[1]/obj_height));
}

Vec2f GL2D::Obj2Win(const float x, const float y) const {
  return Obj2Win(Vec2f(x, y));
}

void GL2D::BitmapString(const string& s, const Vec2f& p,
                        void* font) const {
  glDisable(GL_TEXTURE_2D);
  // Draw white behind
  glColor3f(1.0f, 1.0f, 1.0f);
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      glRasterPos2fv(Win2Obj(Obj2Win(p) + Vec2f(i, j)));
      for (int i = 0; i < s.size(); ++i) {
        glutBitmapCharacter(font, s[i]);
      }
    }
  }

  // Draw black in front
  glColor3f(0.0f, 0.0f, 0.0f);
  glRasterPos2fv(Win2Obj(Obj2Win(p) + Vec2f(0, 0)));
  for (int i = 0; i < s.size(); ++i) {
    glutBitmapCharacter(font, s[i]);
  }
}

void GL2D::BitmapString(const string& s, const Vec2f& obj,
                  int xoff, int yoff,
                  void* font) const {
  glDisable(GL_TEXTURE_2D);
  glColor3f(0.0f, 0.0f, 0.0f);
  Vec2f w = Obj2Win(obj);
  BitmapString(s, Win2Obj(Vec2f(w[0]+xoff, w[1]-yoff)), font);
}

void GL2D::BitmapString(int value, const Vec2f& obj,
                  int xoff, int yoff,
                  void* font) const {
  stringstream ss;
  ss << value;
  BitmapString(ss.str(), obj, xoff, yoff, font);
}

void GL2D::BitmapString(const string& s, const Vec2f& obj,
                  Justify hjustify, Justify vjustify) const {
  glDisable(GL_TEXTURE_2D);
  glColor3f(0.0f, 0.0f, 0.0f);
  Vec2f w = Obj2Win(obj);
  int xoff = 1, yoff = 1;
  if (hjustify == kRightJustify) {
    xoff = - 8 * s.size() - 1;
  } else if (hjustify == kCenterJustify) {
    xoff = - 4 * s.size() - 1;
  }
  if (vjustify == kTopJustify) {
    yoff = - 13 - 1;
  } else if (vjustify == kCenterJustify) {
    // yoff = - 8 - 1;
    yoff = - 6;
  }
  Vec2f p = Win2Obj(Vec2f(w[0]+xoff, w[1]-yoff));
  BitmapString(s, p, GLUT_BITMAP_8_BY_13);
}

void GL2D::BitmapString(int value, const Vec2f& obj,
                  Justify hjustify, Justify vjustify) const {
  stringstream ss;
  ss << value;
  BitmapString(ss.str(), obj, hjustify, vjustify);
}

void GL2D::BitmapString(const string& s) const {
  glDisable(GL_TEXTURE_2D);
  glColor3f(0.0f, 0.0f, 0.0f);
  Vec2f p = Win2Obj(Vec2f(4, window_height-5));
  BitmapString(s, p, GLUT_BITMAP_8_BY_13);
}

Vec3f GL2D::RandomColor() const {
  Vec3f c;
  for (int i = 0; i < 3; ++i) {
    c[i] = random() / static_cast<float>(RAND_MAX);
  }
  return c;
}

// Returned color may not be close to avoid
Vec3f GL2D::RandomColor(const Vec3f& avoid) const {
  static const float DIST_THRESH = .7;
  Vec3f c = RandomColor();
  while ((avoid - c).norm2() < DIST_THRESH) {
    c = RandomColor();
  }
  return c;
}

// Returned color may not be close to avoid
Vec3f GL2D::RandomColor(int seed, const Vec3f& avoid) const {
  srandom(seed+1);
  return RandomColor(avoid);
}

void GL2D::SetColor(int i, const Vec3f& avoid) const {
  // srandom(i+1);
  // Vec3f c = RandomColor(avoid);
  Vec3f c = RandomColor(i, avoid);
  glColor3fv(c);
}

