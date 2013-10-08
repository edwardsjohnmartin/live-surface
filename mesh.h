#ifndef __MESH_H__
#define __MESH_H__

#include <vector>
#include <string>

#include "../vec.h"
#include "./bb.h"
#include "./material.h"
#include "./triangle.h"
#include "./common.h"
#include "./shared_ptr.h"

struct MeshVertexC {
  MeshVertexC() {}
  MeshVertexC(const Vec3f& p_, const Vec3f& n_, const Vec3b& c_)
      : p(p_), n(n_), c(c_, 255) {}
  MeshVertexC(const Vec3f& p_, const Vec3f& n_, const Vec4b& c_)
      : p(p_), n(n_), c(c_) {}
  MeshVertexC(const Vec3f& p_, const Vec3f& n_, const Vec3f& c_)
      : p(p_), n(n_) {
    for (int i = 0; i < 3; ++i)
      c[i] = static_cast<GLbyte>(c_[i]*255);
    c[3] = 255;
  }
  Vec3f p;  // point
  Vec3f n;  // normal
  Vec4b c;  // color
  GLbyte padding[4];  // to make it 32 bytes
};

struct MeshVertex
{
  MeshVertex() {}
  MeshVertex(const Vec3f& p_, const Vec3f& n_)
      : p(p_), n(n_) {}
  Vec3f p;  // point
  Vec3f n;  // normal
  // Vec4b c;  // color
  // GLbyte padding[4];  // to make it 32 bytes
  // GLbyte padding[8];  // to make it 32 bytes
};

class Mesh {
 public:
  Mesh();

  void InitVBO() const;
  void Display(bool wireframe) const;
  void DisplayNormals() const;

  void AddVertex(const Vec3f& v);
  void AddTriangle(const Triangle& t);
  template <typename Iter>
  void AddTriangles(Iter begin, Iter end) {
    for (Iter i = begin; i != end; ++i) {
      AddTriangle(*i);
    }
  }
  void AddMaterial(const Material& m) {
    _materials.push_back(m);
    if (_cur_mtl == -1) _cur_mtl = _materials.size()-1;
  }

  Vec3f Centroid() const;

  const Material& GetMaterial() const { return _materials[0]; }
  void SetMaterial(const Material& m) { _materials[0] = m; }

  void new_material(int material_idx, const std::string& name) {
    _materials.push_back(Material(name));
  }

  void set_cur_material(const string& name) {
    for (int i = 0; i < _materials.size(); ++i) {
      if (_materials[i].name() == name) {
        set_cur_material(i);
      }
    }
  }

  void set_cur_material(int cur_mtl) {
    _cur_mtl = cur_mtl;
  }

  void set_ambient(int material_idx, const Vec3f& ambient) {
    _materials[material_idx].set_ambient(ambient);
  }
  void set_diffuse(int material_idx, const Vec3f& diffuse) {
    _materials[material_idx].set_diffuse(diffuse);
  }
  void set_specular(int material_idx, const Vec3f& specular) {
    _materials[material_idx].set_specular(specular);
  }
  void set_specular_coeff(int material_idx, const float& coeff) {
    _materials[material_idx].set_specular_coeff(coeff);
  }
  void set_texture(int material_idx, const string& texture) {
    _materials[material_idx].set_texture(texture);
  }

  const std::vector<Vec3f>& vertices() const {
    check_lean();
    // return *_vertices;
    // return _vertices->Get();
    return _vertices;
  }

  // Sets the triangles and vertices.  It is possible that there
  // are more vertices than needed, so collapse down to including only
  // those necessary.
  void SetAndCollapse(const vector<Vec3f>& vertices,
                      const vector<Triangle>& triangles);

  const std::vector<Triangle>& triangles() const {
    check_lean();
    return _triangles;
  }

  int num_triangles() const {
    return _num_triangles;
  }

  void OrientPolygons();
  void InvertOrientation();

  const Material& material(int i) const { return _materials[i]; }
  Material& material(int i) { return _materials[i]; }

  const BoundingBox3f& bb() const { return _bb; }
  int num_materials() const { return _materials.size(); }

  void compute_normals();
  const Vec3f& normal(int i) const { return _normals[i]; }

  float pick(const Vec3f& p, const Vec3f& v);

  // Clears just about everything from memory after putting everything
  // on the graphics card.
  void make_lean();
  bool is_lean() const {
    return (_num_triangles != _triangles.size());
  }

  void print_stats() const;

 private:
  void check_lean() const {
    if (is_lean()) {
      throw logic_error("This mesh has been made lean");
    }
  }

 private:
  std::vector<Vec3f> _vertices;
  std::vector<Vec3f> _normals;
  std::vector<Triangle> _triangles;
  std::vector<Material> _materials;
  int _cur_mtl;
  BoundingBox3f _bb;
  int _num_triangles;
  // mutable bool _use_vbo;
  mutable bool _vbo_init;
  mutable GLuint _buffer_names[2];
  
  mutable Vec3f _centroid;
};

#endif
