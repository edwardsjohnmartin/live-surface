#include "./canvas2d.h"

#include <sstream>
#include "./texture.h"
// #include "./dijkstra.h"
#include "./dijkstra2.h"
//#include "AMSComplex2.h"

Canvas2D::Canvas2D(const int win_width, const int win_height,
                   const string& fn)
    : GL2D(win_width, win_height), image(fn),
      dijkstra1(0), dijkstra2(0) {
  seeds.push_back(Vec2f());
  seeds.push_back(Vec2f());

  // Resize image
  Image newImage = image.Half();
  newImage = newImage.Half();
  newImage = newImage.Half();
  newImage = newImage.Half();
  newImage = newImage.Half();
  // image = newImage;
}

Canvas2D::~Canvas2D() {
  if (dijkstra1) {
    delete dijkstra1;
  }
  if (dijkstra2) {
    delete dijkstra2;
  }
}

void Canvas2D::Init() {
  texture_ids = new GLuint[1];
  glGenTextures(1, texture_ids);
  image.LoadImages(texture_ids[0]);
  img_width = image.Width();
  img_height = image.Height();
}

void Canvas2D::DrawSimplifyLines(
    // const SNode& node, const Vec2i& pos, const Vec2i& size) const {
    const SNode& node) const {
  const Vec2i pos = node.Pos();
  const Vec2i size = node.Size();
  glColor3f(1, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2fv(Img2Obj(pos));
  glVertex2fv(Img2Obj(pos+Vec2i(size[0], 0)));
  glVertex2fv(Img2Obj(pos+size));
  glVertex2fv(Img2Obj(pos+Vec2i(0, size[1])));
  glEnd();

  // stringstream ss;
  // ss << node.id();
  // BitmapString(ss.str(), Img2Obj(node.Center()), GLUT_BITMAP_8_BY_13);

  if (node.IsExpanded()) {
    DrawSimplifyLines(node[0]);
    DrawSimplifyLines(node[1]);
    DrawSimplifyLines(node[2]);
    DrawSimplifyLines(node[3]);
  }

  typedef SNode::Nbr_const_iter Iter;
  glColor3f(0, 1, 0);
  glBegin(GL_LINES);
  for (int i = 0; i < 4; ++i) {
    for (Iter it = node.NbrsBegin(i); it != node.NbrsEnd(i); ++it) {
      if (node.id() < (*it)->id()) {
        glColor3f(0, 1, 0);
      } else {
        glColor3f(0, 0, 1);
      }
      const Vec2f p = Img2Obj(node.Center());
      const Vec2f q = Img2Obj((*it)->Center());
      glVertex2fv(p);
      glVertex2fv((p+q)/2);
    }
  }
  glEnd();
}

void Canvas2D::Display() {
  const float r = window_width / static_cast<float>(window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-r, r, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_ids[0]);

  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);

  glTexCoord2d(0, 0);   glVertex3f(-r, 1, 0);
  glTexCoord2d(1, 0);   glVertex3f(r, 1, 0);
  glTexCoord2d(1, 1);   glVertex3f(r, -1, 0);
  glTexCoord2d(0, 1);   glVertex3f(-r, -1, 0);

  glEnd();

  glDisable(GL_TEXTURE_2D);

  // Draw simplify lines
  shared_ptr<Image> imgPtr(new Image(image));
  snode = SNode(imgPtr, Vec2i(), Vec2i(image.Width(), image.Height()));
  snode.Expand();
  snode[0].Expand();
  snode[1].Expand();
  snode[1][0].Expand();
  snode[1][1].Expand();
  snode[0][0].Expand();
  glLineWidth(1);
  glColor3f(1, 0, 0);
  DrawSimplifyLines(snode);

  glPointSize(3);
  glColor3f(0, 0, 0);
  glBegin(GL_POINTS);
  for (int i = 0; i < seeds.size(); ++i) {
    glVertex2fv(Img2Obj(Vec2i(Obj2Img(seeds[i]))));
  }
  glEnd();

  
  vector<Vec2i> path1;
  if (dijkstra1) {
    Vec2f obj = mouse_cur;
    if (seeds.size() > 1)
      obj = seeds[1];
    Vec2f img_pnt = Obj2Img(obj);
    const int imgx = (int)img_pnt[0];
    const int imgy = (int)img_pnt[1];

    dijkstra1->Path(imgx, imgy, back_inserter(path1));
    glColor3f(0, 1, 0);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < path1.size(); ++i) {
      Vec2i v = path1[i];
      Vec2f o = Img2Obj(v);
      glVertex2fv(o);
    }
    glEnd();
  }

  if (dijkstra2) {
//    Vec2f obj = seeds[2];
    // Vec2f obj = mouse_cur;
    // if (seeds.size() > 1)
    //   obj = seeds[1];
//    Vec2f img_pnt = Obj2Img(obj);
//    const int imgx = (int)img_pnt[0];
//    const int imgy = (int)img_pnt[1];

    for (int i = 0; i < path1.size(); ++i) {
      vector<Vec2i> path;
      const Vec2i pt = path1[i];
      dijkstra2->Path(pt[0], pt[1], back_inserter(path));
      glColor3f(0, 0, 1);
      glLineWidth(2);
      glBegin(GL_LINE_STRIP);
      for (int j = 0; j < path.size(); ++j) {
        Vec2i v = path[j];
        Vec2f o = Img2Obj(v);
        glVertex2fv(o);
      }
      glEnd();
    }
  }

  if (!msg.empty()) {
    BitmapString(msg);
  }
}

void Canvas2D::Mouse(int button, int state, int x, int y) {
  if (state == GLUT_DOWN) {
    const Vec2f obj = Win2Obj(Vec2i(x, y));
    const Vec2f img_pnt = Obj2Img(obj);
    const int imgx = img_pnt[0];
    const int imgy = img_pnt[1];
    // const int mod = glutGetModifiers();

    mouse_cur = obj;
    // if (button == GLUT_RIGHT_BUTTON || (mod & GLUT_ACTIVE_CTRL)) {
    if (button == GLUT_LEFT_BUTTON) {
      seeds[0] = obj;
      seeds[1] = obj;
      if (dijkstra1) delete dijkstra1;
      // dijkstra1 = new Dijkstra2(ImageGraph(image));
      dijkstra1 = new Dijkstra2(image);
      dijkstra1->Run(imgx, imgy);
      glutPostRedisplay();
    // } else if (button == GLUT_LEFT_BUTTON) {
    //   seeds[1] = obj;

    //   stringstream ss;
    //   ss << "Image coords: " << imgx << ", " << imgy;
    //   msg = ss.str();

    //   glutPostRedisplay();
    }
  }
}

void Canvas2D::PassiveMouseMotion(int x, int y) {
  if (x < 0 || y < 0 || x >= window_width || y >= window_height)
    return;
  Vec2f obj = Win2Obj(Vec2i(x, y));
  mouse_cur = obj;
  Vec2f img_pnt = Obj2Img(obj);
  const int imgx = img_pnt[0];
  const int imgy = img_pnt[1];

  stringstream ss;
  ss << "Image coords: " << imgx << ", " << imgy;
  msg = ss.str();

  seeds[1] = obj;

  glutPostRedisplay();
}

void Canvas2D::Keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 'r':
      seeds.clear();
      if (dijkstra1) {
        delete dijkstra1;
        dijkstra1 = 0;
      }
      if (dijkstra2) {
        delete dijkstra2;
        dijkstra2 = 0;
      }
      break;
  }
  glutPostRedisplay();
}

Vec2f Canvas2D::Obj2Img(const Vec2f& o) const {
  const GLfloat obj_width = win_obj.size()[0];
  const GLfloat obj_height = win_obj.size()[1];
  const Vec2f offset = o - win_obj.min();
  return Vec2f(
      img_width * (offset[0]/obj_width),
      img_height * (1 - offset[1]/obj_height));
}

Vec2f Canvas2D::Img2Obj(const Vec2i& i) const {
  return Img2Obj(Vec2f(i));
}

Vec2f Canvas2D::Img2Obj(const Vec2f& i) const {
  const GLfloat obj_width = win_obj.size()[0];
  const GLfloat obj_height = win_obj.size()[1];
  return Vec2f((i[0]/img_width)*obj_width,
               (1 - i[1]/img_height)*obj_height) + win_obj.min();
}
