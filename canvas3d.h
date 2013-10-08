#ifndef __MEDIAL3_H__
#define __MEDIAL3_H__

#include <vector>

// #include "../defs.h"
#include "./timer.h"
#include "../vec.h"
// #include "../vertex.h"
#include "./common.h"
#include "./bb.h"
#include "./gl3d.h"
#include "./mesh.h"
// #include "../octree.h"
// #include "../graph.h"

// TODO: Clean up textures
class Medial3 : public GL3D {
 public:
  typedef oct::Timer Timer;
  // typedef oct::Constants::index_t index_t;
  // typedef oct::Constants::level_t level_t;
  // typedef oct::VertexNetwork<3> VertexNetwork;

  // static const int kWidth = oct::Constants::kWidth;
  // static const Vec3f red;

 public:
  Medial3(const int win_width, const int win_height);//,
          // const Vec2f& world_min, const Vec2f& world_max);
  ~Medial3();

  void SaveTransformations();
  void ReadTransformations();
  
  virtual int ProcessArgs(int argc, char** argv);
  virtual void Init();
  virtual void Mouse(int button, int state, int x, int y);
  virtual void MouseMotion(int x, int y);
  virtual void Keyboard(unsigned char key, int x, int y);
  virtual void Special(unsigned char key, int x, int y);
  virtual void Display();

  // Returns the picked point in object coordinates
  virtual Vec3f Pick(int x, int y, bool& hit);
  virtual void Recenter(int x, int y);

  void ResetCenter() {
    center = bb.center();
  }

  void ReadMesh(const std::string& filename, bool medial = false);

  const BoundingBox3f& bbox() const { return bb; }

  Vec3f& RotVec() { return rot_vec; }
  GLfloat& RotAngle() { return rot_angle; }

 private:
  void DrawAxis();
  Vec3f MapMouse(GLfloat x, GLfloat y);

  void UpdateVBO();

 private:
  std::vector<Mesh> meshes;
  BoundingBox3f bb;

  GLfloat mouse_x, mouse_y;
  GLfloat mouse_down_x, mouse_down_y;
  bool left_down;
  bool middle_down;
  bool right_down;
  Vec3f center;
  Vec3f rot_vec;
  GLfloat rot_angle;
  GLfloat rot_matrix[16];// = {1, 0, 0, 0,
                            // 0, 1, 0, 0,
                            // 0, 0, 1, 0,
                            // 0, 0, 0, 1};

  Vec2f mouse_down;
  Vec2f strafe;
  Vec2f down_strafe;

  GLfloat down_zoom;// = 1;

  bool scene_lighting;// = false;

  bool show_mesh;
};

#endif
