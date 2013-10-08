#include "./gl3d.h"

#include <fstream>
#include <sstream>

#include "./common.h"

vector<Vec3f> GL3D::colors;

void GL3D::InitColors() {
  if (colors.empty()) {
    ifstream in("colors.config");
    if (in) {
      while (!in.eof()) {
        float r, g, b;
        in >> r >> g >> b;
        colors.push_back(Vec3f(r, g, b));
      }
      in.close();
    }
  }
}

Vec3f GL3D::RandomColor() {
  Vec3f c;
  for (int i = 0; i < 3; ++i) {
    c[i] = random() / static_cast<float>(RAND_MAX);
  }
  return c;
}

// Returned color may not be close to avoid
Vec3f GL3D::RandomColor(int seed, const Vec3f& avoid) {
  InitColors();
  if (colors.size() > seed) {
    return colors[seed];
  }
  static const float DIST_THRESH = .7;
  srandom(seed+1);
  Vec3f c = RandomColor();
  while ((avoid - c).norm2() < DIST_THRESH) {
    c = RandomColor();
  }
  return c;
}

// Vec3f GL3D::SetColor(int i) {
//   srandom(i+1);
//   Vec3f c = RandomColor();
//   glColor3fv(c);
//   return c;
// }

// Vec3f GL3D::SetColor(int i, const Vec3f& avoid) {
//   srandom(i+1);
//   Vec3f c = RandomColor(avoid);
//   glColor3fv(c);
//   return c;
// }

Vec3f GL3D::Win2Obj(const Vec3f& p) const {
  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];
  GLdouble objX, objY, objZ;
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);
  gluUnProject(p[0], p[1], p[2],
               modelview, projection, viewport,
               &objX, &objY, &objZ);
  return Vec3f(objX, objY, objZ);
}

void GL3D::Reshape(const int win_width, const int win_height) {
  // window_width = w[0];
  // window_height = w[1];
  window_width = win_width;
  window_height = win_height;
  window_aspect = window_width / static_cast<float>(window_height);
}

Vec3f GL3D::Obj2Win(const Vec3f& obj) const {
  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];
  GLdouble winX, winY, winZ;
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);
  gluProject(obj[0], obj[1], obj[2],
             modelview, projection, viewport,
             &winX, &winY, &winZ);
  return Vec3f(winX, winY, winZ);
}

enum Justify { kLeftJustify, kRightJustify,
               kTopJustify, kBottomJustify,
               kCenterJustify };

// p is in window coordinates
void GL3D::BitmapString(const string& s, const Vec2i& p,
                        void* font) const {
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  // // Draw white behind
  // glColor3f(1.0f, 1.0f, 1.0f);
  // for (int i = -1; i <= 1; ++i) {
  //   for (int j = -1; j <= 1; ++j) {
  //     glRasterPos2fv(Win2Obj(Obj2Win(p) + Vec2f(i, j)));
  //     for (int i = 0; i < s.size(); ++i) {
  //       glutBitmapCharacter(font, s[i]);
  //     }
  //   }
  // }

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, window_width, 0, window_height);

  // Draw black in front
  glColor3f(0.0f, 0.0f, 0.0f);
  glRasterPos2iv(p);
  for (int i = 0; i < s.size(); ++i) {
    glutBitmapCharacter(font, s[i]);
  }

  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glEnable(GL_LIGHTING);
}

// p is in object coordinates
void GL3D::BitmapString(const string& s, const Vec3f& p, void* font) const {
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  // Draw white behind
  glColor3f(1.0f, 1.0f, 1.0f);
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      glRasterPos3fv(Win2Obj(Obj2Win(p) + Vec3f(i, j, -.01)));
      for (int i = 0; i < s.size(); ++i) {
        glutBitmapCharacter(font, s[i]);
      }
    }
  }

  // Draw black in front
  glColor3f(0.0f, 0.0f, 0.0f);
  glRasterPos3fv(Win2Obj(Obj2Win(p) + Vec3f(0, 0, -.02)));
  for (int i = 0; i < s.size(); ++i) {
    glutBitmapCharacter(font, s[i]);
  }

  glEnable(GL_LIGHTING);
}

// obj is in world coordinates.
// xoff, yoff are in window coordinates
void GL3D::BitmapString(const string& s, const Vec3f& obj,
                        int xoff, int yoff,
                        void* font) const {
  Vec3f w = Obj2Win(obj);
  Vec3f p = Win2Obj(w + Vec3f(xoff, -yoff, 0));
  BitmapString(s, p, font);
}

void GL3D::BitmapString(int value, const Vec3f& obj,
                        int xoff, int yoff,
                        void* font) const {
  stringstream ss;
  ss << value;
  BitmapString(ss.str(), obj, xoff, yoff, font);
}

void GL3D::BitmapString(const string& s, const Vec3f& obj,
                  Justify hjustify, Justify vjustify) const {
  Vec3f w = Obj2Win(obj);
  int xoff = 1, yoff = 1;
  if (hjustify == kRightJustify) {
    xoff = - 8 * s.size() - 1;
  } else if (hjustify == kCenterJustify) {
    xoff = - 4 * s.size() - 1;
  }
  if (vjustify == kTopJustify) {
    yoff = - 13 - 1;
  } else if (vjustify == kCenterJustify) {
    yoff = - 8 - 1;
  }
  Vec3f p = Win2Obj(w + Vec3f(xoff, 1, 0));
  BitmapString(s, p, GLUT_BITMAP_8_BY_13);
}

void GL3D::BitmapString(int value, const Vec3f& obj,
                  Justify hjustify, Justify vjustify) const {
  stringstream ss;
  ss << value;
  BitmapString(ss.str(), obj, hjustify, vjustify);
}

void GL3D::BitmapString(const string& s) const {
  glDisable(GL_TEXTURE_2D);
  glColor3f(0.0f, 0.0f, 0.0f);
  Vec3f p = Win2Obj(Vec3f(4, window_height-5, 0));
  glRasterPos2f(p[0], p[1]);
  for (int i = 0; i < s.size(); ++i) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);
  }
}

// size = edge length
void GL3D::glCube(const Vec3f& obj, int size) const {
  const Vec3f win = Obj2Win(obj);
  const Vec3f obj_offset =
      Win2Obj(Vec3f(win[0]+size, win[1], win[2]));
  const double off = (obj_offset-obj).norm();
  glTranslatef(obj[0], obj[1], obj[2]);
  glutSolidCube(off);
  glTranslatef(-obj[0], -obj[1], -obj[2]);
}

void GL3D::SetMaterial(const Material& m) const {
  const GLfloat mat_emission[] = { 0, 0, 0, 1 };
  const GLfloat mat_specular[] = { m.specular()[0], m.specular()[1],
                                   m.specular()[2], 1 };
  const GLfloat mat_diffuse[] = { m.diffuse()[0], m.diffuse()[1],
                                  m.diffuse()[2], 1 };
  const GLfloat mat_ambient[] = { m.ambient()[0], m.ambient()[1],
                                  m.ambient()[2], 1 };
  const GLfloat alpha = m.specular_coeff();
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, alpha);
  const int texture = m.texture_id();
  if (texture != -1) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
  } else {
    glDisable(GL_TEXTURE_2D);
  }
}

void GL3D::SetDiffuseAmbient(
    const float r, const float g, const float b) const {
  Vec3f c(r, g, b);
  Material m;
  m.set_diffuse(c);
  m.set_ambient(c);
  SetMaterial(m);
}
