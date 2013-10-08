#include <iostream>
#include <fstream>
#include <vector>
// #include <Magick++.h> 

#include "./vec.h"
#include "./common.h"
#include "./texture.h"
#include "./canvas2d.h"

using namespace std;
// using namespace Magick;

static const int kWindowWidth = 600;
static const int kWindowHeight = 450;
int window_width = kWindowWidth;
int window_height = kWindowHeight;

// Canvas2D canvas(kWindowWidth, kWindowHeight,
//                 "/Users/jedwards/ldata/nordic");
Canvas2D* canvas;

void MyDisplay() {
  glClear(GL_COLOR_BUFFER_BIT);

  canvas->Display();

  glFlush();
  glutSwapBuffers();
}

void Init() {
  glClearColor(1.0, 1.0, 1.0, 1.0);
  canvas->Init();
}

void Mouse(int button, int state, int x, int y) {
  canvas->Mouse(button, state, x, y);
}

void MouseMotion(int x, int y) {
  canvas->MouseMotion(x, y);
}

void PassiveMouseMotion(int x, int y) {
  canvas->PassiveMouseMotion(x, y);
}

void Keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 'q':
      exit(EXIT_SUCCESS);
      break;
    default:
      canvas->Keyboard(key, x, y);
      break;
  }
}

void Special(int key, int x, int y) {
  switch (key) {
    // case GLUT_KEY_DOWN:
    // case GLUT_KEY_RIGHT:
    //   break;
    // case GLUT_KEY_UP:
    // case GLUT_KEY_LEFT:
    //   break;
    case GLUT_KEY_F5:
      glutFullScreen();
      glutPostRedisplay();
      break;
    default:
      canvas->Special(key, x, y);
      break;
  }
}

// Window size changed
void Reshape(int width, int height) {
  window_width = width;
  window_height = height;
  glViewport(0, 0, window_width, window_height);
//  const float r = window_width / static_cast<float>(window_height);

  canvas->Reshape(window_width, window_height);
  Init();
}

int main(int argc, char** argv) {
  string fn = "/Users/jedwards/ldata/nordic-small/nordic";
  if (argc > 1) {
    fn = argv[1];
  }

  canvas = new Canvas2D(kWindowWidth, kWindowHeight,
                fn);

  ifstream sizein("size.config");
  if (sizein) {
    sizein >> window_width >> window_height;
    sizein.close();
  }

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(window_width, window_height);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Live Surface");
  glutDisplayFunc(MyDisplay);
  glutKeyboardFunc(Keyboard);
  glutSpecialFunc(Special);
  glutMouseFunc(Mouse);
  glutMotionFunc(MouseMotion);
  glutPassiveMotionFunc(PassiveMouseMotion);
  glutReshapeFunc(Reshape);

  Init();
  glutMainLoop();
}
