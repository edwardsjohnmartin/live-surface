#include <iostream>
#include <vector>
#include <set>
#include <climits>

#include "./mesh.h"
#include "../orientation.h"

using namespace std;

namespace {
#define BUFFER_OFFSET(i) ((char*)0 + (i))

Vec3f Normal(const Vec3f& a, const Vec3f& b, const Vec3f& c) {
  return (b-a)^(c-a);
}

}

// Taken from
// http://nehe.gamedev.net/tutorial/vertex_buffer_objects/22002/
bool IsExtensionSupported(const char* szTargetExtension)
{
  const unsigned char *pszExtensions = NULL;
  const unsigned char *pszStart;
  unsigned char *pszWhere, *pszTerminator;

  // Extension names should not have spaces
  pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
  if( pszWhere || *szTargetExtension == '\0' )
    return false;

  // Get Extensions String
  pszExtensions = glGetString( GL_EXTENSIONS );
  if (!pszExtensions) {
    return false;
    cout << "no extensions" << endl;
  }

  // Search The Extensions String For An Exact Copy
  pszStart = pszExtensions;
  for(;;)
  {
    pszWhere = (unsigned char *) strstr(
        (const char *) pszStart, szTargetExtension);
    if( !pszWhere )
      break;
    pszTerminator = pszWhere + strlen( szTargetExtension );
    if( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
      if( *pszTerminator == ' ' || *pszTerminator == '\0' )
        return true;
    pszStart = pszTerminator;
  }
  return false;
}

Mesh::Mesh() : _num_triangles(0) {
  _vbo_init = false;
  _cur_mtl = -1;

  _buffer_names[0] = 0;
  _buffer_names[1] = 0;
}

void Mesh::InitVBO() const {
  _vbo_init = true;
  bool use_vbo = IsExtensionSupported("GL_ARB_vertex_buffer_object");
  if (!use_vbo) {
    cerr << "**********************" << endl;
    cerr << " VBO not supported " << endl;
    cerr << "**********************" << endl;
    return;
  }

  glDeleteBuffers(2, _buffer_names);
  glGenBuffers(2, _buffer_names);

  // Send vertex buffer object
  // MeshVertex* mverts = new MeshVertex[_vertices->size()];
  // for (int i = 0; i < _vertices->size(); ++i) {
  //   mverts[i] = MeshVertex(_vertices->at(i), normal(i), Vec4b(255, 0, 0, 255));
  // }
  // glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
  // glBufferData(
  //     GL_ARRAY_BUFFER, _vertices->size()*sizeof(MeshVertex),
  //     mverts, GL_STREAM_DRAW);
  // delete [] mverts;
  MeshVertex* mverts = new MeshVertex[_vertices.size()];
  for (int i = 0; i < _vertices.size(); ++i) {
    mverts[i] = MeshVertex(_vertices[i], normal(i));
  }
  glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
  glBufferData(
      GL_ARRAY_BUFFER, _vertices.size()*sizeof(MeshVertex),
      mverts, GL_STREAM_DRAW);
  delete [] mverts;

  // Send index buffer object
  GLuint* indices = new GLuint[_triangles.size()*3];
  for (int i = 0; i < _triangles.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      indices[i*3+j] = _triangles[i][j];
    }
  }
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[1]);
  glBufferData(
      // GL_ELEMENT_ARRAY_BUFFER, _polygons.size()*3*sizeof(GLuint),
      GL_ELEMENT_ARRAY_BUFFER, _triangles.size()*3*sizeof(GLuint),
      indices, GL_STREAM_DRAW);
  delete [] indices;
}

void Mesh::Display(bool wireframe) const {
  if (!_vbo_init) {
    InitVBO();
  }

  // glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
  // glVertexPointer(3, GL_FLOAT, sizeof(MeshVertex), 0);
  // glNormalPointer(GL_FLOAT, sizeof(MeshVertex),
  //                 BUFFER_OFFSET(3*sizeof(GLfloat)));
  // glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MeshVertex),
  //                BUFFER_OFFSET(6*sizeof(GLfloat)));

  glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
  glVertexPointer(3, GL_FLOAT, sizeof(MeshVertex), 0);
  glNormalPointer(GL_FLOAT, sizeof(MeshVertex),
                  BUFFER_OFFSET(3*sizeof(GLfloat)));

  // glBindBuffer(GL_ARRAY_BUFFER, _vertices->buffer_name());
  // glVertexPointer(3, GL_FLOAT, sizeof(MeshVertex), 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[1]);

  // Rendering
  glEnableClientState(GL_VERTEX_ARRAY);
  // glEnableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);

  if (wireframe) {
    glLineWidth(1.0f);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(.2f, .2f, .2f);
    glDisableClientState(GL_COLOR_ARRAY);
    glDrawElements(GL_TRIANGLES, _num_triangles*3, GL_UNSIGNED_INT, 0);
    // glEnableClientState(GL_COLOR_ARRAY);
  } else {
    // Material properties
    const Material& m = _materials[0];
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, Vec4f(0, 0, 0, 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Vec4f(m.specular(), 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vec4f(m.diffuse(), 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, Vec4f(m.ambient(), 1));
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, m.specular_coeff());

    // Surface
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glPolygonMode(GL_FRONT, GL_FILL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1.0);
    glDrawElements(GL_TRIANGLES, _num_triangles*3, GL_UNSIGNED_INT, 0);
    glDisable(GL_POLYGON_OFFSET_FILL);
  }

  glDisableClientState(GL_NORMAL_ARRAY);
  // glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void Mesh::DisplayNormals() const {
  // glDisable(GL_LIGHTING);
  // glLineWidth(2);
  // glColor3f(1, 0, 0);
  // glBegin(GL_LINES);
  // for (int i = 0; i < _vertices->size(); ++i) {
  //   const Vec3f& vertex = _vertices->at(i);
  //   Vec3f n = normal(i) / 2;
  //   glVertex3fv(vertex);
  //   glVertex3fv(vertex+n);
  // }
  // glEnd();
  // glEnable(GL_LIGHTING);
}

// void Mesh::set_vertices(oct::shared_ptr<std::vector<Vec3f> >& vertices) {
// void Mesh::set_vertices(oct::shared_ptr<MeshVertices>& vertices) {
//   _vertices = vertices;
//   for (int i = 0; i < vertices->size(); ++i) {
//     _bb(vertices->at(i));
//   }
// }

void Mesh::AddVertex(const Vec3f& v) {
  // _vertices->push_back(v);
  _vertices.push_back(v);
  _bb(v);
}

void Mesh::SetAndCollapse(const vector<Vec3f>& vertices,
                          const vector<Triangle>& triangles) {
  _vertices.clear();
  vector<int> v2v(vertices.size(), -1);
  for (int i = 0; i < triangles.size(); ++i) {
    const Triangle& t = triangles[i];
    for (int j = 0; j < 3; ++j) {
      const int old_idx = t[j];
      int new_idx = v2v[old_idx];
      if (new_idx == -1) {
        new_idx = _vertices.size();
        v2v[old_idx] = new_idx;
        _vertices.push_back(vertices[old_idx]);
      }
    }
    _triangles.push_back(Triangle(v2v[t[0]], v2v[t[1]], v2v[t[2]]));
  }
  _num_triangles = _triangles.size();
  _vbo_init = false;
}

// void Mesh::AddTextureVertex(const Vec3f& v) {
//   _texture_vertices.push_back(v);
// }

// void Mesh::AddPolygon(const std::vector<int>& p, const std::vector<int>& pt) {
//   _polygons.push_back(p);
//   _polygons_tex.push_back(pt);
//   _polygon2material.push_back(_cur_mtl);

//   // for (int i = 0; i < p.size(); ++i) {
//   //   _bb(_vertices->at(i));
//   // }
// }

// void Mesh::AddTriangle(const Triangle& t) {
//   vector<int> p;
//   for (int i = 0; i < 3; ++i) p.push_back(t[i]);
//   _polygons.push_back(p);
//   // _polygons_tex.push_back(pt);
//   _polygon2material.push_back(_cur_mtl);

//   // for (int i = 0; i < p.size(); ++i) {
//   //   _bb(_vertices->at(i));
//   // }
// }
void Mesh::AddTriangle(const Triangle& t) {
  _triangles.push_back(t);
  _num_triangles++;
}

struct MyPair {
  MyPair(const Vec3f& v_, const int i_) : v(v_), i(i_) {}
  bool operator<(const MyPair& rhs) const {
    return v < rhs.v;
  }
  bool operator==(const MyPair& rhs) const {
    return v == rhs.v;
  }
  Vec3f v;
  int i;
};

// void Mesh::MergeDuplicateVertices() {
//   vector<int> v2v(_vertices->size());
//   set<MyPair> added;
//   int idx = 0;
//   for (int i = 0; i < _vertices->size(); ++i) {
//     const MyPair p(_vertices->at(i), idx);
//     if (added.find(p) == added.end()) {
//       added.insert(p);
//       v2v[i] = idx++;
//     } else {
//       v2v[i] = added.find(p)->i;
//     }
//   }

//   vector<Vec3f> new_vertices(added.size());
//   typedef set<MyPair>::const_iterator Iter;
//   for (Iter it = added.begin(); it != added.end(); ++it) {
//     new_vertices.at(it->i) = it->v;
//   }
//   *_vertices = new_vertices;

//   // Update triangles
//   for (int i = 0; i < _polygons.size(); ++i) {
//     vector<int>& polygon = _polygons[i];
//     for (int j = 0; j < polygon.size(); ++j) {
//       polygon[j] = v2v[polygon[j]];
//     }
//   }
// }

// void Mesh::GetMergeMap(vector<int>& v2v,
//                        // vector<Vec3f>& new_vertices) const {
//                        MeshVertices& new_vertices) const {
//   // vector<int> v2v(_vertices->size());
//   v2v.resize(_vertices->size());
//   set<MyPair> added;
//   int idx = 0;
//   for (int i = 0; i < _vertices->size(); ++i) {
//     const MyPair p(_vertices->at(i), idx);
//     if (added.find(p) == added.end()) {
//       added.insert(p);
//       v2v[i] = idx++;
//     } else {
//       v2v[i] = added.find(p)->i;
//     }
//   }

//   // vector<Vec3f> new_vertices(added.size());
//   new_vertices.resize(added.size());
//   typedef set<MyPair>::const_iterator Iter;
//   for (Iter it = added.begin(); it != added.end(); ++it) {
//     new_vertices.at(it->i) = it->v;
//   }
// }

// void Mesh::UpdateTrianglesWithVertexMap(const vector<int>& v2v) {
//   // Update triangles
//   // for (int i = 0; i < _polygons.size(); ++i) {
//   //   vector<int>& polygon = _polygons[i];
//   //   for (int j = 0; j < polygon.size(); ++j) {
//   //     polygon[j] = v2v[polygon[j]];
//   //   }
//   // }
//   for (int i = 0; i < _triangles.size(); ++i) {
//     Triangle& t = _triangles[i];
//     _triangles[i] = Triangle(v2v[t[0]], v2v[t[1]], v2v[t[2]]);
//   }
// }

void Mesh::compute_normals() {
  // vector<vector<int> > v2p(_vertices->size());
  // for (int i = 0; i < _polygons.size(); ++i) {
  //   const vector<int>& polygon = _polygons[i];
  //   for (int j = 0; j < polygon.size(); ++j) {
  //     v2p[polygon[j]].push_back(i);
  //   }
  // }
  vector<vector<int> > v2p(_vertices.size());
  for (int i = 0; i < _triangles.size(); ++i) {
    const Triangle& t = _triangles[i];
    for (int j = 0; j < 3; ++j) {
      v2p[t[j]].push_back(i);
    }
  }

  _normals.resize(_vertices.size());
  for (int i = 0; i < _vertices.size(); ++i) {
    // Loop through each adjacent polygon and find
    // its weighted normal
    Vec3f n = Vec3f::zero();
    for (int j = 0; j < v2p[i].size(); ++j) {
      // const vector<int>& p = _polygons[v2p[i][j]];
      const Triangle& p = _triangles[v2p[i][j]];
      n += Normal(
          _vertices.at(p[0]), _vertices.at(p[1]), _vertices.at(p[2]));
    }
    // _normals[i] = Normalize(n);
    _normals[i] = n.unit();
  }
}

// Vec3f Mesh::normal_from_polygon(const vector<int>& polygon) const {
//   return Normal(_vertices->at(polygon[0]),
//                 _vertices->at(polygon[1]),
//                 _vertices->at(polygon[2])).unit();

//   // vector<vector<int> > v2p(_vertices.size());
//   // for (int i = 0; i < _polygons.size(); ++i) {
//   //   const vector<int>& polygon = _polygons[i];
//   //   for (int j = 0; j < polygon.size(); ++j) {
//   //     v2p[polygon[j]].push_back(i);
//   //   }
//   // }

//   // // _normals.resize(_vertices.size());
//   // for (int i = 0; i < _vertices.size(); ++i) {
//   //   // Loop through each adjacent polygon and find
//   //   // its weighted normal
//   //   Vec3f n = Vec3f::zero();
//   //   for (int j = 0; j < v2p[i].size(); ++j) {
//   //     const vector<int>& p = _polygons[v2p[i][j]];
//   //     n += Normal(_vertices[p[0]], _vertices[p[1]], _vertices[p[2]]);
//   //   }
//   //   _normals[i] = Normalize(n);
//   // }
// }

void Mesh::OrientPolygons() {
  check_lean();
  // _triangles = orient(_triangles);
  orient_lean(_triangles);
  if (_num_triangles != _triangles.size()) {
    cerr << "num_triangles changed in orient: " << _num_triangles
         << " " << _triangles.size() << endl;
  }
  _num_triangles = _triangles.size();

  // vector<Triangle> triangles;
  // for (int i = 0; i < _polygons.size(); ++i) {
  //   const vector<int>& P = _polygons[i];
  //   triangles.push_back(Triangle(P[0], P[1], P[2]));
  // }

  // triangles = orient(triangles);
  // // triangles = orient_new(triangles);
  // // triangles = split(triangles, *_vertices);

  // _polygons.clear();
  // for (int i = 0; i < triangles.size(); ++i) {
  //   vector<int> P;
  //   const Triangle& t = triangles[i];
  //   for (int j = 0; j < 3; ++j) {
  //     P.push_back(t[j]);
  //   }
  //   _polygons.push_back(P);
  // }
}

void Mesh::InvertOrientation() {
  check_lean();
  // for (int i = 0; i < _polygons.size(); ++i) {
  //   reverse(_polygons[i].begin(), _polygons[i].end());
  // }
  for (int i = 0; i < _triangles.size(); ++i) {
    _triangles[i] = _triangles[i].inverted();
  }
  for (int i = 0; i < _normals.size(); ++i) {
    _normals[i] = _normals[i] * -1;
  }
  _vbo_init = false;
}

float Mesh::pick(const Vec3f& p, const Vec3f& v) {
  check_lean();
  float mint = 999999999;
  // for (int i = 0; i < _polygons.size(); ++i) {
  for (int i = 0; i < _triangles.size(); ++i) {
    // Get the plane
    // const Vec3f P = _vertices.at(_polygons[i][0]);
    // const Vec3f Q = _vertices.at(_polygons[i][1]);
    // const Vec3f R = _vertices.at(_polygons[i][2]);
    const Vec3f P = _vertices.at(_triangles[i][0]);
    const Vec3f Q = _vertices.at(_triangles[i][1]);
    const Vec3f R = _vertices.at(_triangles[i][2]);
    const Vec3f n = ((Q-P)^(R-P)).unit();
    const float D = -n*P;
    // assert(-n*Q == D);
    if (fabs(-n*Q - D) > 0.0001) throw logic_error("nqd");
    // (p+tv)*n+D=0
    // t = -(p*n+D)/v*n
    if (v*n != 0) {
      const float t = -(p*n+D)/(v*n);
      const Vec3f C = p+t*v;
      if (((Q-P).unit()^(C-P).unit())*n > 0 &&
          ((R-Q).unit()^(C-Q).unit())*n > 0 &&
          ((P-R).unit()^(C-R).unit())*n > 0) {
        if (t < mint) {
          mint = t;
        }
      }
    }
  }
  return mint;
}

// void Mesh::shrink_to_fit() {
//   const int n = _vertices.size();
//   oct::shared_ptr<std::vector<Vec3f> > new_vertices(new vector<Vec3f>(n));
//   for (int i = 0; i < _vertices.size(); ++i) {
//     new_vertices.at(i) = _vertices.at(i);
//   }
//   _vertices = new_vertices;
// }

void Mesh::make_lean() {
  if (!_vbo_init) {
    InitVBO();
  }
  // _vertices.clear();
  // _normals.clear();
  // _triangles.clear();
  // _vertices.reset(new vector<Vec3f>());
  // _vertices.reset(new MeshVertices());
  _vertices = vector<Vec3f>();
  _normals = vector<Vec3f>();
  _triangles = vector<Triangle>();
}

void Mesh::print_stats() const {
  const int vu = (_vertices.capacity()*sizeof(Vec3f));
  const int nu = (_normals.capacity()*sizeof(Vec3f));
  const int tu = (_triangles.capacity()*sizeof(Triangle));
  const int mu = (_materials.capacity()*sizeof(Material));

  cout << "vertices size = " << _vertices.size() << endl;
  cout << "vertices capacity = " << _vertices.capacity() << endl;
  cout << "vertices usage = " << vu << endl;

  cout << "normals size = " << _normals.size() << endl;
  cout << "normals capacity = " << _normals.capacity() << endl;
  cout << "normals usage = " << nu << endl;

  cout << "triangles size = " << _triangles.size() << endl;
  cout << "triangles capacity = " << _triangles.capacity() << endl;
  cout << "triangles usage = " << tu << endl;

  cout << "materials usage = " << mu << endl;

  cout << "total usage = " << (vu+nu+tu+mu) << endl;
}

Vec3f Mesh::Centroid() const {
  if (_centroid == Vec3f()) {
    Vec3f c_sum;
    float area_sum = 0;
    for (int i = 0; i < _triangles.size(); ++i) {
      const Vec3f P = _vertices.at(_triangles[i][0]);
      const Vec3f Q = _vertices.at(_triangles[i][1]);
      const Vec3f R = _vertices.at(_triangles[i][2]);
      const Vec3f c = (P + Q + R)/3;
      const float area = ((Q-P)^(R-P)).norm()/2;
      c_sum += (c * area);
      area_sum += area;
    }
    _centroid = c_sum / area_sum;
  }
  return _centroid;
}
