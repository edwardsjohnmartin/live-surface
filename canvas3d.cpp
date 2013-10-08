#include "./canvas3d.h"

#include <map>
#include <climits>
#include <algorithm>

#include "./io.h"

// Vec3f center;
GLdouble mymodelview[16];

namespace {
struct MergePair {
  MergePair(const Vec3f& v_, const int i_) : v(v_), i(i_) {}
  bool operator<(const MergePair& rhs) const {
    return v < rhs.v;
  }
  bool operator==(const MergePair& rhs) const {
    return v == rhs.v;
  }
  Vec3f v;
  int i;
};

// Merges identical vertices and returns a map from old index to new index.
void GetMergeMap(vector<int>& v2v,
                 const vector<Vec3f>& vertices,
                 vector<Vec3f>& new_vertices) {
  v2v.resize(vertices.size());
  set<MergePair> added;
  int idx = 0;
  for (int i = 0; i < vertices.size(); ++i) {
    const MergePair p(vertices[i], idx);
    if (added.find(p) == added.end()) {
      added.insert(p);
      v2v[i] = idx++;
    } else {
      v2v[i] = added.find(p)->i;
    }
  }

  new_vertices.resize(added.size());
  typedef set<MergePair>::const_iterator Iter;
  for (Iter it = added.begin(); it != added.end(); ++it) {
    new_vertices.at(it->i) = it->v;
  }
}

  void UpdateTrianglesWithVertexMap(const vector<int>& v2v,
                                    vector<Triangle>& triangles) {
  for (int i = 0; i < triangles.size(); ++i) {
    Triangle& t = triangles[i];
    triangles[i] = Triangle(v2v[t[0]], v2v[t[1]], v2v[t[2]]);
  }
}

}

struct M3Callback {
  M3Callback(const Medial3* m3_) : m3(m3_) {}
  const Medial3* m3;
};

struct DrawVertexDistanceLineCallback : public M3Callback {
  DrawVertexDistanceLineCallback(const Medial3* m3_) : M3Callback(m3_) {}
  bool operator()(const int vi, const Vec3i& p) {
    return m3->DrawVertexDistanceLine(vi, p);
  }
};

struct DrawLabelCallback : public M3Callback {
  DrawLabelCallback(const Medial3* m3_) : M3Callback(m3_) {}
  bool operator()(const int vi, const Vec3i& p) {
    return m3->DrawLabel(vi, p);
  }
};

struct DrawIDCallback : public M3Callback {
  DrawIDCallback(const Medial3* m3_) : M3Callback(m3_) {}
  bool operator()(const int vi, const Vec3i& p) {
    return m3->DrawID(vi, p);
  }
};

struct DrawEdgeCallback : public M3Callback {
  DrawEdgeCallback(const Medial3* m3_) : M3Callback(m3_) {}
  bool operator()(const int vi, const int n_vi,
                  const Vec3i& p, const Vec3i& q,
                  const oct::Direction<3>& d) {
    return m3->DrawEdge(vi, n_vi, p, q, d);
  }
};

struct OctreeEdgeCallback : public M3Callback {
  OctreeEdgeCallback(const Medial3* m3_)
      : M3Callback(m3_), vn(m3_->GetVertices()),
        vertices(new vector<Vec3i>()),
        edges(new vector<Edge>()),
        labels(new vector<int>()),
        alphas(new vector<int>()),
        vi2vi(new vector<int>(m3_->GetVertices().size())) {}
  bool operator()(const int vi, const Vec3i& p) {
    vi2vi->at(vi) = vertices->size();
    vertices->push_back(p);
    labels->push_back(vn.Label(vi));
    for (int i = 0; i < 3; ++i) {
      const int n_vi = vn.Neighbor(vi, oct::Direction<3>::FromAxis(i, true));
      if (n_vi != -1) {
        edges->push_back(Edge(vi, n_vi));
      }
    }

    alphas->push_back(vn.Alpha(vi));
    return true;
  }
  void UpdateIndices() {
    for (int i = 0; i < edges->size(); ++i) {
      Edge& e = edges->at(i);
      e = Edge(vi2vi->at(e[0]), vi2vi->at(e[1]));
    }
  }
  const oct::VertexNetwork<3>& vn;
  oct::shared_ptr<vector<Vec3i> > vertices;
  oct::shared_ptr<vector<Edge> > edges;
  oct::shared_ptr<vector<int> > labels;
  oct::shared_ptr<vector<int> > alphas;
  oct::shared_ptr<vector<int> > vi2vi;
};

Medial3::Medial3(const int win_width, const int win_height)//,
                 // const Vec2f& world_min, const Vec2f& world_max)
    : GL3D(win_width, win_height), texture_ids(0) {//, world_min, world_max) {
  mouse_x = 0;
  mouse_y = 0;
  mouse_down_x = 0;
  mouse_down_y = 0;
  left_down = false;
  middle_down = false;
  right_down = false;
  rot_vec = Vec3f(1, 0, 0);
  rot_angle = 0;
  memset(rot_matrix, 0, 16 * sizeof(GLfloat));
  for (int i = 0; i < 4; ++i) rot_matrix[4*i+i] = 1;
  // rot_matrix[16] = {1, 0, 0, 0,
  //                           0, 1, 0, 0,
  //                           0, 0, 1, 0,
  //                           0, 0, 0, 1};

  // maxDepth = 5;

  down_zoom = 1;

  scene_lighting = false;

  show_mesh = true;
  show_octree = false;
  show_vertices = false;
  show_medial_separator = true;

  show_vertex_distance_lines = false;
  show_all_vertex_distance_lines = false;
  show_vertex_id = false;
  show_axis = false;
  show_cell_id = false;
  show_statistics = true;

  o = oct::OctreeOptions::For3D();
  o.medial_subdivide_max = -1;

  mesh_mode = 2;
  medial_mode = 3;

  _buffers_valid = false;

  zoom = 1.5;
  _target_object = 0;
  _exploded_factor = 0.01;
  _explode_dir = 0;
  _explode_mode = 0;
  _max_ring = 1;
  _shot_base = "shot";
  _path_size = .03;
}

Medial3::~Medial3() {
  if (texture_ids) {
    delete [] texture_ids;
  }
}

Vec3i Medial3::Obj2Oct(const Vec3f& v) const {
  const Vec3f size = bb_objects.size();
  const float max_size = max(size[0], max(size[1], size[2]));
  return
      Vec3i(Vec3f(kWidth, kWidth, kWidth).prod((v-bb_objects.min())/max_size));
}

Vec3f Medial3::Oct2Obj(const Vec3i& v) const {
  const Vec3f vf(v[0], v[1], v[2]);
  const Vec3f size = bb_objects.size();
  const float max_size = max(size[0], max(size[1], size[2]));
  const GLfloat w = kWidth;
  return (vf/w)*max_size+bb_objects.min();
}

GLfloat Medial3::Oct2Obj(int dist) const {
  const Vec3f size = bb_objects.size();
  const float max_size = max(size[0], max(size[1], size[2]));
  const GLfloat ow = kWidth;
  return (dist/ow)*max_size;
}

bool Medial3::DrawVertexDistanceLine(const int vi, const Vec3i& p) const {
  if (show_all_vertex_distance_lines) {
    glLineWidth(1.0);
    glBegin(GL_LINES);
    // All secondary in yellow
    glColor3f(0.7, 0.7, 0);
    oct::Vertex<3> v = vertices[vi];
    typedef oct::Vertex<3>::OPConstIter Iter;
    for (Iter it = v.OtherPointsBegin(); it != v.OtherPointsEnd(); ++it) {
      const Vec3i& cp = vertices.ClosestPointByPointId(*it);
      glVertex3fv(Oct2Obj(p));
      glVertex3fv(Oct2Obj(cp));
    }
    glEnd();
  }

  glLineWidth(2.0);
  glBegin(GL_LINES);
  // Closest in blue
  const Vec3i& cp = vertices.ClosestPoint(vi);
  glColor3f(0, 0, 0.7);
  glVertex3fv(Oct2Obj(p));
  glVertex3fv(Oct2Obj(cp));
  glEnd();

  Vec3f c(0, 0, .8);
  glColor3fv(c);
  Material m;
  m.set_diffuse(c);
  m.set_ambient(c);
  SetMaterial(m);
  Vec3f obj = Oct2Obj(cp);
  const Vec3f win = Obj2Win(obj);
  const Vec3f obj_offset = Win2Obj(Vec3f(win[0]+5, win[1], win[2]));
  const double off = (obj_offset-obj).norm();
  glTranslatef(obj[0], obj[1], obj[2]);
  glutSolidCube(off);
  glTranslatef(-obj[0], -obj[1], -obj[2]);

  // stringstream ss;
  // ss << vertices.ClosestPointIndex(vi);
  // BitmapString(ss.str(), Oct2Obj(cp));

  return true;
}

void Medial3::DrawVertexDistanceLines() {
  glDisable(GL_LIGHTING);
  glColor3f(0, 0.7, 0);
  glLineWidth(1.0);
  // glBegin(GL_LINES);
  oct::VisitVertices<3>(vertices, DrawVertexDistanceLineCallback(this));
  // glEnd();
  glEnable(GL_LIGHTING);
}

bool Medial3::DrawLabel(const int vi, const Vec3i& p) const {
  static const double base_size = 10;
  static const int max_dist = oct::Constants::kWidth / 2;

  if (vertices.ClosestPointIndex(vi) != -1) {
    const int dist = (vertices.ClosestPoint(vi) - p).norm();
    const double dist_normalized = dist/static_cast<double>(max_dist);

    const int label = vertices.Label(vi);
    const Vec3f red(1, 0, 0);
    // Vec3f c(SetColor(label, red));
    Vec3f c = RandomColor(label, red);
    glColor3fv(c);
    Material m;
    m.set_diffuse(c);
    m.set_ambient(c);
    SetMaterial(m);

    const Vec3f obj = Oct2Obj(p);
    const Vec3f win = Obj2Win(obj);
    const Vec3f obj_offset =
        Win2Obj(Vec3f(win[0]+base_size*dist_normalized+5, win[1], win[2]));
    const double off = (obj_offset-obj).norm();
    glTranslatef(obj[0], obj[1], obj[2]);
    glutSolidCube(off);
    glTranslatef(-obj[0], -obj[1], -obj[2]);
  }

  return true;
}

void Medial3::DrawVertexLabels() {
  // glColor3f(0, 0, 0);
  // VisitVertices(vertices, DrawLabelCallback(this));

  bool use_vbo = IsExtensionSupported("GL_ARB_vertex_buffer_object");
  if (!use_vbo) {
    cerr << "VBOs not supported -- can't draw octree" << endl;
    return;
  }

  if (!_buffers_valid) {
    UpdateVBO();
  }

  glLineWidth(1.0);
  glColor3f(0.0, 0.0, 0.0);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_LIGHTING);

  glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[2]);
  glVertexPointer(3, GL_FLOAT, sizeof(MeshVertexC), 0);
  glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MeshVertexC),
                 (char*)0 + 6*sizeof(GLfloat));
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[3]);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);

  glDrawElements(GL_TRIANGLES, vertices.size()*36, GL_UNSIGNED_INT, 0);

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  // glBegin(GL_LINES);
  // for (int i = 0; i < cb.edges->size(); ++i) {
  //   glVertex3fv(Oct2Obj((*cb.vertices)[(*cb.edges)[i][0]]));
  //   glVertex3fv(Oct2Obj((*cb.vertices)[(*cb.edges)[i][1]]));
  // }
  // glEnd();

  glEnable(GL_LIGHTING);
}

bool Medial3::DrawID(const int vi, const Vec3i& p) const {
  // if (vi == 452 || vi == 120726 || vi == 206 || vi == 455 || vi == 1268)
  BitmapString(vi, Oct2Obj(p), kCenterJustify, kCenterJustify);
  return true;
}

void Medial3::DrawVertexIDs() {
  VisitVertices(vertices, DrawIDCallback(this));
}

bool Medial3::DrawSeparatorVertex(const int vi, const int n_vi,
                         const Vec3i& p, const Vec3i& q,
                         const oct::Direction<3>& d) const {
  if (!vertices.IsBaseOfLeafCell(vi)) return true;

  // vector<pair<Vec3i, Edge> > intersections;
  vector<pair<Vec3i, oct::LabeledSegment<3> > > intersections;
  oct::GetIntersectionsAroundFace<3>(vi, p, vertices.LeafCellLevel(vi), 0, 1,
                                     vertices,
                                     back_inserter(intersections), o);

  for (int i = 0; i < intersections.size(); ++i) {
    Vec3f obj = Oct2Obj(intersections[i].first);
    const Vec3f win = Obj2Win(obj);
    const Vec3f obj_offset =
        Win2Obj(Vec3f(win[0]+5, win[1], win[2]));
    const double off = (obj_offset-obj).norm();
    glTranslatef(obj[0], obj[1], obj[2]);
    glutSolidCube(off);
    glTranslatef(-obj[0], -obj[1], -obj[2]);
  }

  return true;
}

void Medial3::DrawMesh(const Mesh& mesh, const bool face_normals) {
  mesh.Display(false);
}

void Medial3::DrawMesh(const Mesh& mesh,
                       const bool surface, const bool wireframe,
                       const bool face_normals) {
  if (surface)
    mesh.Display(false);
  if (wireframe)
    mesh.Display(true);
  // mesh.DisplayNormals();
}

void Medial3::DrawMeshes() {
  typedef set<int> Set;
  typedef set<int>::const_iterator Set_iter;

  glPolygonMode(GL_FRONT, GL_FILL);
  // glPolygonMode(GL_BACK, GL_POINT);
  glPolygonMode(GL_BACK, GL_FILL);
  const double d = _exploded_factor;
  // for (int i = 0; i < meshes.size(); ++i) {
  int outer_ri = 0;
  Set prev_ring;
  Set ring = gvd_graph->Ring(_target_object, outer_ri);
  vector<Vec3d> dirs(meshes.size());
  while (!ring.empty()) {
    for (Set_iter it = ring.begin(); it != ring.end(); ++it) {
      const int i = *it;
      bool draw = (i == _target_object);
      if (!draw) {
        draw = (_explode_mode == 0 || _explode_mode == 2);
      }
      if (!draw) {
        // const Set& one = gvd_graph->Ring(_target_object, _max_ring);
        // draw = (one.find(i) != one.end());
        const int ri = gvd_graph->WhichRing(_target_object, i);
        if (_explode_mode == 1 || _explode_mode == 3)
          draw = (ri <= _max_ring);
        else if (_explode_mode == 4)
          draw = (ri == _max_ring);
      }
      if (draw) {
        glPushMatrix();
        Vec3d dir;
        if (_explode_mode > 1 && _explode_mode < 4 && i != _target_object) {
          const int ri = gvd_graph->WhichRing(_target_object, i);
          if (ri == 1) {
            if (_explode_dir == 0) {
              dir = gvd_graph->Dir(_target_object, i) * d * ri;
            } else {
              dir = (medial_meshes[i].Centroid()
                     -medial_meshes[_target_object].Centroid()).unit() * d * ri;
            }
          } else {
            const Set& one = gvd_graph->Ring(i, 1);
            vector<int> parents;
            set_intersection(one.begin(), one.end(),
                             prev_ring.begin(), prev_ring.end(),
                             back_inserter(parents));
            Vec3d dir_sum;
            for (int j = 0; j < parents.size(); ++j) {
              const int pi = parents[j];
              const Vec3d pdir = dirs[pi] * gvd_graph->GetArea(pi, i);
              dir_sum += pdir;
            }
            if (_explode_dir == 0) {
              dir = dir_sum.unit() * d * ri;
            } else {
              dir = (medial_meshes[i].Centroid()
                     -medial_meshes[_target_object].Centroid()).unit() * d * ri;
            }
          }
          glTranslated(dir[0], dir[1], dir[2]);
        }
        dirs[i] = dir;
        const Material m = meshes[i].GetMaterial();
        if (i == _target_object && _explode_mode > 0) {
          meshes[i].SetMaterial(Material::FromDiffuseAmbient(Vec3f(1, 0, 0)));
        }
        DrawMesh(meshes[i], mesh_mode&2, mesh_mode&1);
        if (i == _target_object) {
          meshes[i].SetMaterial(m);
        }
      }
      if (!meshes[i].is_lean()) {
        //meshes[i].make_lean();
      }
      glPopMatrix();
    }
    ++outer_ri;
    prev_ring = ring;
    ring = gvd_graph->Ring(_target_object, outer_ri);
  }
}

void Medial3::ComputeMedialSurface() {
  vector<Vec3i> m_vertices;
  map<Vec3i, int> m_vertex2vid;
  vector<vector<Triangle> > m_triangles(meshes.size());
  // map<int, int> base2centroid;
  map<int, Centroids> base2centroids;
  // maps centroid index to a set of triangles
  map<int, set<Triangle> > centroid2tris;
  // map<int, map<Edge, set<Triangle> > > centroid2tris;
  // TopoGraph_h gvd_graph(new TopoGraph());
  gvd_graph.reset(new TopoGraph());
  GraphConstructor3_h bisector_graph(new GraphConstructor3());
  TileInfo_h info(new TileInfo(
      vertices, m_vertices, m_vertex2vid, m_triangles, base2centroids,
      centroid2tris, gvd_graph, bisector_graph, o));
  // SearchOct v(vertices, m_vertices, m_vertex2vid, m_triangles, base2centroid,
  //             centroid2tris, gvd_graph, bisector_graph, o);
  SearchOct v(info);
  oct::Timer t("Computing medial intersections");
  oct::VisitVertices<3>(vertices, v);
  t.restart("Outside faces");

  // Do outside faces
  const int max_width = oct::Constants::kWidth;
  for (int i = 0; i < 3; ++i) {
    Vec3i base_point(0, 0, 0);
    base_point[i] = max_width;
    const int base_vi = vertices.FindNeighbor(
        0, oct::Direction<3>::FromAxis(i, true), 0);
    const int axes_arr[] = {(i+1)%3, (i+2)%3};
    const vector<int> axes(axes_arr, axes_arr+2);
    int max_width = oct::Constants::kWidth;
    // SearchQuad v(vertices, m_vertices, m_vertex2vid, m_triangles,
    //              -1, base2centroid, centroid2tris, i, false,
    //              gvd_graph, bisector_graph, o);
    static Centroids invalid;
    SearchQuad v(info, invalid, i, false);
    oct::VisitVertices<3>(vertices, base_point, axes, max_width-1, base_vi, v);
  }

  // Move centroids to center of mass points
  typedef map<int, set<Triangle> >::const_iterator Iter;
  typedef set<Triangle>::const_iterator Set_iter;
  t.restart("Updating centroids");
  vector<Vec3i> new_vertices = m_vertices;
  for (Iter it = centroid2tris.begin(); it != centroid2tris.end(); ++it) {
    const int ci = it->first;
    const set<Triangle>& triangles = it->second;
    Vec3d c;
    for (Set_iter t_it = triangles.begin(); t_it != triangles.end(); ++t_it) {
      const Triangle& t = *t_it;
      for (int i = 0; i < 3; ++i) {
        if (t[i] != ci) {
          c += m_vertices[t[i]];
        }
      }
    }
    if (c[0] < 0 || c[1] < 0 || c[2] < 0) {
      cerr << "c = " << c << endl;
    }
    c = c / (triangles.size()*2);
    bisector_graph->ReplacePoint(m_vertices[ci], c);
    new_vertices[ci] = c;
    m_vertices[ci] = c;
  }

  vector<vector<Triangle> > new_triangles = m_triangles;

  medial_meshes = vector<Mesh>(meshes.size());

  // material
  t.restart("Material and duplicate vertices");
  for (int i = 0; i < medial_meshes.size(); ++i) {
    medial_meshes[i].AddMaterial(meshes[i].material(0));
  }

  // vertices
  vector<Vec3f> vertices;
  for (int i = 0; i < m_vertices.size(); ++i) {
    vertices.push_back(Oct2Obj(new_vertices[i]));
  }

  // triangles
  vector<int> v2v;
  vector<Vec3f> merged_vertices;
  GetMergeMap(v2v, vertices, merged_vertices);
  vertices = merged_vertices;
  typedef pair<int, int> Pair;
  const std::map<Pair, list<Triangle> >& directed_tris = *info->directed_tris;
  for (int i = 0; i < medial_meshes.size(); ++i) {
    UpdateTrianglesWithVertexMap(v2v, new_triangles[i]);
    medial_meshes[i].SetAndCollapse(vertices, new_triangles[i]);

    typedef std::set<int> Set;
    typedef Set::const_iterator Set_iter;
    typedef list<Triangle> List;
    typedef List::const_iterator List_iter;
    const Set& nbrs = gvd_graph->at(i);
    for (Set_iter it = nbrs.begin(); it != nbrs.end(); ++it) {
      const int j = *it;
      pair<int, int> e(i, j);
      const List& dir_tris = directed_tris.find(e)->second;
      Vec3d sum;
      double area_sum = 0;
      for (List_iter lit = dir_tris.begin(); lit != dir_tris.end(); ++lit) {
        const Triangle& t = *lit;
        const Vec3d n =
            (vertices[t[1]]-vertices[t[0]])^(vertices[t[2]]-vertices[t[0]]);
        sum += n;
        area_sum += n.norm()/2;
      }
      const Vec3d n = sum.unit();
      gvd_graph->SetDir(i, j, n);
      gvd_graph->SetArea(i, j, area_sum);
    }
  }  

  for (int i = 0; i < medial_meshes.size(); ++i) {
    medial_meshes[i].OrientPolygons();
    medial_meshes[i].compute_normals();
    bb_full += medial_meshes[i].bb();
  }
  // center = bb_full.center();
  if (center == Vec3f())
    ResetCenter();

  medial_graph = bisector_graph->GetGraph();
  std::vector<Vec3d>& mgv = medial_graph.GetVertices();
  for (int i = 0; i < mgv.size(); ++i) {
    mgv[i] = Oct2Obj(mgv[i]);
  }
}

void Medial3::DrawMedialSeparator() {
  glPolygonMode(GL_FRONT, GL_FILL);
  // glPolygonMode(GL_BACK, GL_POINT);
  //glPolygonMode(GL_BACK, GL_POINT);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glPointSize(0.0f);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  if (show_medial_separator) {
    for (int i = 0; i < medial_meshes.size(); ++i) {
      DrawMesh(medial_meshes[i], medial_mode&2, medial_mode&1, true);
    }
  }
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  glLineWidth(1);
  glDisable(GL_CULL_FACE);

  glLineWidth(3.0);
  glColor3f(0.0, 0.0, 1.0);
  if (_path_size < 0.01) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < search_path.size(); ++i) {
      glVertex3dv(medial_graph[search_path[i]]);
    }
    glEnd();
  } else if (!search_path.empty()) {
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, Vec4f(0, 0, 0, 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Vec4f(.9, .9, .9, 1));
    // glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vec4f(0, 1, 1, 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vec4f(0, 1, 0, 1));
    // glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, Vec4f(.2, .2, .4, 1));
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, Vec4f(0, .4, 0, 1));
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 40);
    Vec3f cur;
    for (int i = 0; i < search_path.size()-1; ++i) {
      const Vec3f& p = medial_graph[search_path[i]];
      const Vec3f& q = medial_graph[search_path[i+1]];
      const Vec3f v = (q-p).unit();
      const float len = (q-p).norm();
      float f = 0;
      if ((p-cur).norm() < _path_size*3) {
        f = _path_size*3 - (p-cur).norm();
      }
      for (; f < len; f += _path_size*3) {
        const Vec3f r = p + v*f;
        glTranslatef(r[0], r[1], r[2]);
        glutSolidSphere(_path_size, 10, 10);
        glTranslatef(-r[0], -r[1], -r[2]);
      }
    }
  }
}

bool Medial3::DrawEdge(const int vi, const int n_vi,
                       const Vec3i& p, const Vec3i& q,
                       const oct::Direction<3>& d) const {
  glVertex3fv(Oct2Obj(p));
  glVertex3fv(Oct2Obj(q));
  return true;
}

struct OctVertex {
  OctVertex() {}
  OctVertex(const Vec3f& p_)
      : p(p_) {}
  Vec3f p;  // point
  GLbyte padding[4];  // to make it 16 bytes
  // Vec3f n;  // normal
  // Vec4b c;  // color
  // GLbyte padding[4];  // to make it 32 bytes
};

void Medial3::UpdateVBO() {
  OctreeEdgeCallback cb(this);
  VisitVertices(vertices, cb);
  cb.UpdateIndices();

  // static bool initialized = false;
  // static GLuint _buffer_names[2];
  // glDeleteBuffers(4, _buffer_names);
  glGenBuffers(4, _buffer_names);
  _buffers_valid = true;

  {
    //----------------------------------------
    // Octree lines
    //----------------------------------------
    // Send vertex buffer object
    const int n = cb.vertices->size();
    OctVertex* mverts = new OctVertex[n];
    for (int i = 0; i < n; ++i) {
      mverts[i] = OctVertex(Oct2Obj(cb.vertices->at(i)));
    }
    glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
    glBufferData(
        GL_ARRAY_BUFFER, n*sizeof(OctVertex),
        mverts, GL_STREAM_DRAW);
    delete [] mverts;

    // Send index buffer object
    const int m = cb.edges->size();
    _num_octree_edges = m;
    GLuint* indices = new GLuint[m*2];
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < 2; ++j) {
        indices[i*2+j] = cb.edges->at(i)[j];
      }
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[1]);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER, m*2*sizeof(GLuint),
        indices, GL_STREAM_DRAW);
    delete [] indices;
  }

  //----------------------------------------
  // Octree vertices
  //----------------------------------------
  // Send vertex buffer object
  const int n = cb.vertices->size();
  MeshVertexC* mverts = new MeshVertexC[n*8];
  const Vec3f norm;
  Vec3f red(1, 0, 0);
  for (int i = 0; i < n; ++i) {
    const Vec3i v = cb.vertices->at(i);
    const int d = cb.alphas->at(i) / 8;
    const int l = cb.labels->at(i);
    const Vec3f c = RandomColor(l, red);
    mverts[i*8+0] = MeshVertexC(Oct2Obj(v+Vec3i(-d, -d, -d)), norm, c);
    mverts[i*8+1] = MeshVertexC(Oct2Obj(v+Vec3i(d, -d, -d)), norm, c);
    mverts[i*8+2] = MeshVertexC(Oct2Obj(v+Vec3i(d, d, -d)), norm, c);
    mverts[i*8+3] = MeshVertexC(Oct2Obj(v+Vec3i(-d, d, -d)), norm, c);
    mverts[i*8+4] = MeshVertexC(Oct2Obj(v+Vec3i(-d, -d, d)), norm, c);
    mverts[i*8+5] = MeshVertexC(Oct2Obj(v+Vec3i(d, -d, d)), norm, c);
    mverts[i*8+6] = MeshVertexC(Oct2Obj(v+Vec3i(d, d, d)), norm, c);
    mverts[i*8+7] = MeshVertexC(Oct2Obj(v+Vec3i(-d, d, d)), norm, c);
    // mverts[i*8+0] = MeshVertex(Oct2Obj(v+Vec3i(-d, -d, -d)), norm);
    // mverts[i*8+1] = MeshVertex(Oct2Obj(v+Vec3i(d, -d, -d)), norm);
    // mverts[i*8+2] = MeshVertex(Oct2Obj(v+Vec3i(d, d, -d)), norm);
    // mverts[i*8+3] = MeshVertex(Oct2Obj(v+Vec3i(-d, d, -d)), norm);
    // mverts[i*8+4] = MeshVertex(Oct2Obj(v+Vec3i(-d, -d, d)), norm);
    // mverts[i*8+5] = MeshVertex(Oct2Obj(v+Vec3i(d, -d, d)), norm);
    // mverts[i*8+6] = MeshVertex(Oct2Obj(v+Vec3i(d, d, d)), norm);
    // mverts[i*8+7] = MeshVertex(Oct2Obj(v+Vec3i(-d, d, d)), norm);
  }
  glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[2]);
  glBufferData(
      GL_ARRAY_BUFFER, n*8*sizeof(MeshVertexC),
      mverts, GL_STREAM_DRAW);
  delete [] mverts;

  // Send index buffer object
  GLuint* indices = new GLuint[n*36];
  static const int inds[] = { 0, 1, 3,
                              3, 1, 2,
                              1, 5, 2,
                              2, 5, 6,
                              5, 4, 6,
                              6, 4, 7,
                              4, 0, 7,
                              7, 0, 3,
                              3, 2, 7,
                              7, 2, 6,
                              1, 0, 4,
                              1, 4, 5 };
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 36; ++j) {
      indices[i*36+j] = inds[j] + i*8;
    }
  }
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[3]);
  glBufferData(
      GL_ELEMENT_ARRAY_BUFFER, n*36*sizeof(GLuint),
      indices, GL_STREAM_DRAW);
  delete [] indices;
}

void Medial3::DrawOctree() {
  bool use_vbo = IsExtensionSupported("GL_ARB_vertex_buffer_object");
  if (!use_vbo) {
    cerr << "VBOs not supported -- can't draw octree" << endl;
    return;
  }

  if (!_buffers_valid) {
    UpdateVBO();
  }

  glLineWidth(1.0);
  glColor3f(0.0, 0.0, 0.0);

  glDisable(GL_LIGHTING);

  glBindBuffer(GL_ARRAY_BUFFER, _buffer_names[0]);
  glVertexPointer(3, GL_FLOAT, sizeof(OctVertex), 0);
  // glNormalPointer(GL_FLOAT, sizeof(OctVertex),
  //                 BUFFER_OFFSET(3*sizeof(GLfloat)));
  // glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MeshVertex),
  //                BUFFER_OFFSET(6*sizeof(GLfloat)));
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer_names[1]);

  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);

  // glDrawElements(GL_LINES, cb.edges->size()*2, GL_UNSIGNED_INT, 0);
  glDrawElements(GL_LINES, _num_octree_edges*2, GL_UNSIGNED_INT, 0);

  glDisableClientState(GL_VERTEX_ARRAY);

  // glBegin(GL_LINES);
  // for (int i = 0; i < cb.edges->size(); ++i) {
  //   glVertex3fv(Oct2Obj((*cb.vertices)[(*cb.edges)[i][0]]));
  //   glVertex3fv(Oct2Obj((*cb.vertices)[(*cb.edges)[i][1]]));
  // }
  // glEnd();

  glEnable(GL_LIGHTING);

  // glDisable(GL_LIGHTING);
  // glBegin(GL_LINES);
  // VisitEdges(vertices, DrawEdgeCallback(this));
  // glEnd();
  // glEnable(GL_LIGHTING);
}

void Medial3::PrintStatistics() const {
  int num_cells = 0;
  oct::Constants::level_t max_level = 0;
  for (int i = 0; i < vertices.size(); ++i) {
    if (vertices.IsBaseOfLeafCell(i)) {
      ++num_cells;
      max_level = max(max_level, vertices.LeafCellLevel(i));
    }
  }
  {
    stringstream ss;
    ss << "Max level: " << (int)max_level;
    BitmapString(ss.str(), Vec2i(2, 2));
  } {
    stringstream ss;
    ss << "Octree cells: " << num_cells;
    BitmapString(ss.str(), Vec2i(2, 19));
  } {
    if (_picked != Vec3f()) {
      stringstream ss;
      ss << "Picked: " << _picked << " | " << Obj2Oct(_picked);
      BitmapString(ss.str(), Vec2i(2, 19+17));
    }
  }
}

void Medial3::Display() {
  glShadeModel(GL_SMOOTH);
  // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // // glPolygonMode(GL_FRONT, GL_FILL);
  // // glPolygonMode(GL_BACK, GL_LINES);
  // glPolygonMode(GL_BACK, GL_FILL);

  glEnable(GL_LIGHT0);
  // glEnable(GL_LIGHT1);
  // const GLfloat light0_position[] = { -150, 150, 300, 1 };
  const GLfloat light0_ambient[] = { 0.1, 0.1, 0.1, 1 };
  const GLfloat light0_diffuse[] = { 0.6, 0.6, 0.6, 1 };
  const GLfloat light0_specular[] = { 0.3, 0.3, 0.3, 1 };
  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light0_specular);

  glEnable(GL_NORMALIZE);

  // const Vec3f center = bb.center();

  Vec3f size = bb_full.size();
  const Vec3f eye = Vec3f(
      0, 0, size[2]/2 + ((size[1]/2)/tan(20*M_PI/180.0))*1.1) * zoom;
  const Vec3f off = -Vec3f(strafe[0] * size[0],
                           strafe[1] * size[1],
                           0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // gluLookAt(eye[0], eye[1], eye[2],
  //           0, 0, 0,
  //           0, 1, 0);
  gluLookAt(off[0]+eye[0], off[1]+eye[1], off[2]+eye[2],
            off[0], off[1], off[2],
            0, 1, 0);

  // Position the light now so it is always in the same place relative
  // to the camera.
  const Vec3f light0 =
      Vec3f(eye[0]-size[0]/2, eye[1]+size[1]/2, eye[2]+size[2]/2);
  const Vec3f light1 =
      Vec3f(eye[0]+size[0]/2, eye[1]-size[1]/2, eye[2]-size[2]/2);

  // if (!scene_lighting) {
    glLightfv(GL_LIGHT0, GL_POSITION, light0);
    glLightfv(GL_LIGHT1, GL_POSITION, light1);
  // }

  glRotatef(rot_angle*180.0/M_PI, rot_vec[0], rot_vec[1], rot_vec[2]);
  glMultMatrixf(rot_matrix);

  // Position light here if we want it in the same place
  // relative to the world.
  // if (scene_lighting) {
  //   glLightfv(GL_LIGHT0, GL_POSITION, light);
  // }

  glDisable(GL_LIGHTING);
  glLineWidth(4);
  if (show_axis) {
    DrawAxis();
  }
  // glEnable(GL_LIGHTING);

  glTranslatef(-center[0], -center[1], -center[2]);

  if (show_octree) {
    DrawOctree();
  }
  // if (show_medial_separator) {
    DrawMedialSeparator();
  // }
  if (show_mesh) {
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    DrawMeshes();
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  }
  glGetDoublev(GL_MODELVIEW_MATRIX, mymodelview);
  // glDisable(GL_LIGHTING);
  // glColor3f(0, 0, 1);
  // glBegin(GL_LINES);
  // glVertex3fv(p_);
  // glVertex3fv(p_+v_);
  // glEnd();
  // glCube(p_, 5);
  // glColor3f(0, 1, 0);
  // glCube(q_, 15);
  // glEnable(GL_LIGHTING);

  if (show_vertex_distance_lines) {
    DrawVertexDistanceLines();
  }
  if (show_vertex_id) {
    DrawVertexIDs();
  }
  if (show_vertices) {
    DrawVertexLabels();
  }
  if (show_statistics) {
    PrintStatistics();
  }

  // glFlush();
  // glutSwapBuffers();
}

typedef oct::Constants::index_t index_t;
void ProgressBarCallback(const Vec3i& base_point,
                     const index_t level,
                     const index_t max_level,
                     const bool complete) {
  static const index_t w2 = oct::CellWidth(1);
  if (complete) {
    cout << endl;
  } else if (level == 1) {
    if (base_point == Vec3i(w2, 0)) {
      cout << "25%" << ".";
    } else if (base_point == Vec3i(0, w2)) {
      cout << "50%" << ".";
    } else if (base_point == Vec3i(w2, w2)) {
      cout << "75%" << ".";
    }
  } else if (level == 2) {
    cout << ".";
  }
  cout.flush();
}

void Medial3::ReadMesh(const string& filename, bool medial) {
  // Parse the obj file, compute the normals, read the textures
  Mesh mesh;
  if (!ParseObj(filename, mesh)) {
    cerr << "Error: File " << filename << " does not exist. Skipping it."
         << endl;
    return;
  }
  // mesh.print_stats();

  mesh.compute_normals();

  if (texture_ids) {
    delete [] texture_ids;
  }
  texture_ids = new GLuint[mesh.num_materials()];
  glGenTextures(mesh.num_materials(), texture_ids);

  for (int i = 0; i < mesh.num_materials(); ++i) {
    Material& material = mesh.material(i);
    material.LoadTexture(texture_ids[i]);
  }

  Vec3f red(1, 0, 0);
  // if (mesh.polygon2material(0) == -1) {
    Vec3f c = RandomColor(meshes.size(), red);
    Material mat = Material::FromDiffuseAmbient(c);
    if (medial) {
      mat = Material::FromDiffuseAmbient(red);
    }
    mesh.AddMaterial(mat);
  // }

  // mesh.InitVBO();
  if (medial) {
    medial_meshes.clear();
    medial_meshes.push_back(mesh);
    // medial_meshes[0] = mesh;
  } else {
    meshes.push_back(mesh);
  }
  bb_objects += mesh.bb();
  bb_full += mesh.bb();
  // center = bb_full.center();
  if (center == Vec3f())
    ResetCenter();

  // mesh.make_lean();
}

void Medial3::GenerateSurface(const oct::OctreeOptions& o_) {
  // Create the vertices and triangles
  vector<vector<Vec3f> > all_vertices(meshes.size());
  vector<vector<Triangle> > all_triangles(meshes.size());
  for (int i = 0; i < meshes.size(); ++i) {
    const Mesh& mesh = meshes[i];
    all_vertices[i] = mesh.vertices();
    all_triangles[i].insert(all_triangles[i].end(),
                            mesh.triangles().begin(), mesh.triangles().end());
    // const std::vector<std::vector<int> >& polygons = mesh.polygons();
    // for (int j = 0; j < polygons.size(); ++j) {
    //   const std::vector<int>& polygon = polygons[j];
    //   if (polygon.size() > 3) {
    //     throw logic_error(
    //         "Only triangulated surfaces are currently supported");
    //   }
    //   all_triangles[i].push_back(
    //       Triangle(polygon[0], polygon[1], polygon[2]));
    // }
  }

  // cout << "Creating octree to depth " << maxDepth << endl;
  // cout << "Total width = " << oct::Constants::kWidth << endl;

  // Create octree with all_vertices and all_triangles
  // vertices = oct::BuildOctree(all_vertices, all_triangles, maxDepth);
  // vertices = oct::BuildOctree<3, Triangle, oct::LabeledGeometry3>(
  //     all_vertices, all_triangles, maxDepth, ProgressBarCallback);
  // oct::OctreeOptions o = oct::OctreeOptions::For3D();
  const bool lock_subdivide = (o.medial_subdivide_max == -1);
  if (lock_subdivide) o.medial_subdivide_max = o.max_level;
  vertices = oct::BuildOctree<3, Triangle, oct::LabeledGeometry3>(
      all_vertices, all_triangles, o, ProgressBarCallback);
  if (lock_subdivide) o.medial_subdivide_max = -1;
  // cout << "vertices size = " << vertices.size() << endl;
  // Timer t("Computing medial surface");

  ComputeMedialSurface();

  // t.stop();

  if (_buffers_valid) {
    glDeleteBuffers(4, _buffer_names);
  }
  _buffers_valid = false;
}

void Medial3::SaveTransformations() {
  ofstream out("view.config");
  if (out) {
    for (int i = 0; i < 16; ++i) {
      out << rot_matrix[i] << " ";
    }
    out << endl;
    for (int i = 0; i < 3; ++i) {
      out << center[i] << " ";
    }
    out << endl;
    for (int i = 0; i < 2; ++i) {
      out << strafe[i] << " ";
    }
    out << endl;
    out << zoom;
    out << endl;
  }
  out.close();
}

void Medial3::ReadTransformations() {
  ifstream in("view.config");
  if (in) {
    for (int i = 0; i < 16; ++i) {
      in >> rot_matrix[i];
    }
    for (int i = 0; i < 3; ++i) {
      in >> center[i];
    }
    for (int i = 0; i < 2; ++i) {
      in >> strafe[i];
    }
    in >> zoom;
    in.close();
  }
}

void PrintKeyCommands();

int Medial3::ProcessArgs(int argc, char** argv) {
  if (argc < 2) {
    cout << endl;
    cout << "Usage: ./octree3 [-l maxlevel] [-s screenshotBasename] -x "
         << "filename.obj" << endl;
    cout << "  -x - default to not draw medial separator" << endl;
    cout << endl;
    exit(0);
  }

  PrintKeyCommands();

  ReadTransformations();

  int i = 1;
  // if (argc > 1) {
  bool stop = false;
  while (i < argc && !stop) {
    stop = true;
    if (strcmp(argv[i], "-l") == 0) {
      ++i;
      o.max_level = atoi(argv[i]);
      ++i;
      stop = false;
    } else if (strcmp(argv[i], "-s") == 0) {
      ++i;
      _shot_base = argv[i];
      ++i;
      stop = false;
    } else if (strcmp(argv[i], "-x") == 0) {
      show_medial_separator = false;
      ++i;
      stop = false;
    } 
  }
  int num_tris = 0;
  for (; i < argc; ++i) {
    string filename(argv[i]);
    cout << "Reading " << filename << endl;
    ReadMesh(filename);
    num_tris += meshes[meshes.size()-1].num_triangles();
  }

  cout << "Number of objects: " << meshes.size() << endl;
  cout << "Number of triangles: " << num_tris << endl;

  o.simple_q = true;
  GenerateSurface(o);

  return 0;
}

void Medial3::Init() {
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LEQUAL);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  // resize the window
  // window_aspect = window_width/static_cast<float>(window_height);

  // glMatrixMode(GL_PROJECTION);
  // glLoadIdentity();
  // gluPerspective(40.0, window_aspect, 1, 1500);
}

void Medial3::DrawAxis() {
  const Vec3f c(0, 0, 0);
  const float L = 20;
  const Vec3f X(L, 0, 0), Y(0, L, 0), Z(0, 0, L);

  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3fv(c.x);
  glVertex3fv((c+X).x);
  glColor3f(0, 1, 0);
  glVertex3fv(c.x);
  glVertex3fv((c+Y).x);
  glColor3f(0, 0, 1);
  glVertex3fv(c.x);
  glVertex3fv((c+Z).x);
  glEnd();
}

Vec3f Medial3::MapMouse(GLfloat x, GLfloat y) {
  if (x*x + y*y > 1) {
    const GLfloat len = sqrt(x*x + y*y);
    x = x/len;
    y = y/len;
  }
  const GLfloat z = sqrt(max(0.0f, 1 - x*x - y*y));
  return Vec3f(x, y, z);
}

static const int ROTATE_BUTTON = GLUT_LEFT_BUTTON;
static const int STRAFE_BUTTON = GLUT_RIGHT_BUTTON;
static const int ZOOM_BUTTON = GLUT_MIDDLE_BUTTON;
static const GLfloat zfactor = 2;

Vec3f Medial3::Pick(int x, int y, bool& hit) {
  Vec2f m(x, window_height-y);
  // In window coordinates, z==1 --> far
  //                        z==0 --> near

  glMatrixMode(GL_MODELVIEW_MATRIX);
  glLoadMatrixd(mymodelview);

  Vec3f p = Win2Obj(Vec3f(m, 0));
  Vec3f v = Win2Obj(Vec3f(m, 1)) - p;
  float mint = 99999;
  Vec3f picked;
  if (show_mesh) {
    for (int i = 0; i < meshes.size(); ++i) {
      if (!meshes[i].is_lean()) {
        const float t = meshes[i].pick(p, v);
        mint = std::min(mint, t);
      }
    }
  }
  if (show_medial_separator) {
    for (int i = 0; i < medial_meshes.size(); ++i) {
      const float t = medial_meshes[i].pick(p, v);
      mint = std::min(mint, t);
    }
  }
  Vec3f q;
  hit = (mint < 99999);
  if (hit) {
    q = p+mint*v;
  }
  return q;
}

void Medial3::Recenter(int x, int y) {
  bool hit;
  const Vec3f p = Pick(x, y, hit);
  if (hit) {
    center = p;
    cout << "Center updated to " << center << endl;
  }
}

void Medial3::Search(const int start, const int end) {
  search_path.clear();
  medial_graph.Dijkstra(start, end, back_inserter(search_path));
  reverse(search_path.begin(), search_path.end());
  glutPostRedisplay();
}

void Medial3::SetStartSearch(int x, int y) {
  bool hit;
  const Vec3f p = Pick(x, y, hit);
  if (!hit) return;

  double min_dist = numeric_limits<double>::max();
  int start = -1;
  const std::vector<Vec3d>& vertices = medial_graph.GetVertices();
  for (int i = 0; i < vertices.size(); ++i) {
    const double d = (p-vertices[i]).norm2();
    if (d < min_dist) {
      min_dist = d;
      start = i;
    }
  }
  if (start > -1) {
    if (search_path.empty()) {
      search_path.push_back(start);
    } else {
      Search(start, search_path.back());
    }
  }
  // cout << "started search with " << start << endl;
}

void Medial3::SetEndSearch(int x, int y) {
  bool hit;
  const Vec3f p = Pick(x, y, hit);
  if (!hit) return;

  double min_dist = numeric_limits<double>::max();
  int end = -1;
  const std::vector<Vec3d>& vertices = medial_graph.GetVertices();
  for (int i = 0; i < vertices.size(); ++i) {
    const double d = (p-vertices[i]).norm2();
    if (d < min_dist) {
      min_dist = d;
      end = i;
    }
  }
  if (end > -1) {
    if (search_path.empty()) {
      search_path.push_back(end);
    } else {
      Search(search_path.front(), end);
    }
  }
  // cout << "ended search with " << end << endl;
}

void Medial3::Mouse(int button, int state, int x, int y) {
  mouse_x = 2 * x / (GLfloat)window_width - 1;
  mouse_y = 2 * (window_height-y) / (GLfloat)window_height - 1;
  if (button == ROTATE_BUTTON) {
    if (state == GLUT_DOWN) {
      if (!glutGetModifiers()) {
        left_down = true;
        mouse_down_x = mouse_x;
        mouse_down_y = mouse_y;
        mouse_down = Vec2f(mouse_x, mouse_y);
      // } else if (glutGetModifiers() == GLUT_ACTIVE_ALT) {
      // doesn't seem to work
      //   Recenter(x, y);
      } else if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) {
        SetStartSearch(x, y);
      } else if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
        bool hit;
        const Vec3f p = Pick(x, y, hit);
        if (hit)
          _picked = p;
        else
          _picked = Vec3f();
      }
    } else {
      left_down = false;
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glRotatef(rot_angle*180.0/M_PI, rot_vec[0], rot_vec[1], rot_vec[2]);
      glMultMatrixf(rot_matrix);
      glGetFloatv(GL_MODELVIEW_MATRIX, rot_matrix);
      rot_angle = 0;

    }
  } else if (button == STRAFE_BUTTON) {
    if (state == GLUT_DOWN) {
      if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) {
        SetEndSearch(x, y);
      } else if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
        Recenter(x, y);
      } else {
        middle_down = true;
        mouse_down_x = mouse_x;
        mouse_down_y = mouse_y;
        mouse_down = Vec2f(mouse_x, mouse_y);
        down_strafe = strafe;
      }
    } else {
      middle_down = false;
    }
  } else if (button == ZOOM_BUTTON) {
    if (state == GLUT_DOWN) {
      right_down = true;
      mouse_down_x = mouse_x;
      mouse_down_y = mouse_y;
      mouse_down = Vec2f(mouse_x, mouse_y);
      down_zoom = zoom;
    } else {
      right_down = false;
    }
  } else if ((button == 3) || (button == 4)) {
    // It's a wheel event
    // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
    if (state == GLUT_UP) {
      // Disregard redundant GLUT_UP events
    } else {
      // 3 is scroll up, 4 is scroll down
      if (button == 3)
        zoom = zoom * 1.1;
      else
        zoom = zoom * 0.9;
    }
  }

  glutPostRedisplay();
}

void Medial3::MouseMotion(int x, int y) {
  mouse_x = 2 * x / (GLfloat)window_width - 1;
  mouse_y = 2 * (window_height-y) / (GLfloat)window_height - 1;
  Vec2f mouse(mouse_x, mouse_y);
  if (left_down) {
    const Vec3f down_v = MapMouse(mouse_down_x, mouse_down_y);
    const Vec3f v = MapMouse(mouse_x, mouse_y);
    rot_vec = (down_v ^ v).unit();
    rot_angle = acos((down_v * v) / v.norm());
  } else if (middle_down) {
    strafe = down_strafe + (mouse - mouse_down);
  } else if (right_down) {
    zoom = down_zoom * pow(zfactor, mouse_y - mouse_down_y);
  }

  glutPostRedisplay();
}

void PrintKeyCommands() {
  cout << endl;
  cout << "Key commands:" << endl;
  cout << "  g - show mesh geometry" << endl;
  cout << "  o - show octree" << endl;
  cout << "  m - show medial separator" << endl;
  cout << "  G - mesh geometry mode" << endl;
  cout << "  M - medial separator mode" << endl;
  cout << "  d - show vertex distance lines" << endl;
  cout << "  v - show vertices colored by label" << endl;
  cout << "  i - show vertex IDs" << endl;
  cout << "  a - show axis" << endl;
  cout << "  z - zoom in" << endl;
  cout << "  Z - zoom out" << endl;
  cout << "  w - write medial obj file" << endl;
  cout << "  V - toggle full subdivide" << endl;
  cout << "  B - toggle make buffer" << endl;
  cout << "  p - toggle statistics display" << endl;
  cout << "  e - cycle explode mode" << endl;
  cout << "  E - cycle explode direction" << endl;
  cout << "  Y - save rotated views to png" << endl;
  cout << "  t - save current view to view.config" << endl;
  cout << "  c/C - adjust near clipping plane" << endl;
  cout << "  R - reset center" << endl;
  cout << "  r - reset view" << endl;
  cout << "  P - write png image" << endl;
  cout << endl;
  cout << "Mouse commands:" << endl;
  cout << "  left - rotate" << endl;
  cout << "  right - strafe" << endl;
  cout << "  middle - zoom" << endl;
  cout << "  shift+left - begin path" << endl;
  cout << "  shift+right - end path" << endl;
  cout << "  ctrl+left - pick location" << endl;
  cout << "  ctrl+right - set pick as center of rotation" << endl;
  cout << endl;
}

void Medial3::Keyboard(unsigned char key, int x, int y) {
  const float zoom_factor = 1.2;
  bool redisplay = true;
  switch (key) {
    case '0':
      _max_ring = 0;
      break;
    case '1':
      _max_ring = 1;
      break;
    case '2':
      _max_ring = 2;
      break;
    case '3':
      _max_ring = 3;
      break;
    case '4':
      _max_ring = 4;
      break;
    case '5':
      _max_ring = 5;
      break;
    case '6':
      _max_ring = 6;
      break;
    case 'o':
      show_octree = !show_octree;
      break;
    case 'g':
      show_mesh = !show_mesh;
      break;
    case 'm':
      show_medial_separator = !show_medial_separator;
      break;
    case 'G':
      mesh_mode = (mesh_mode%3) + 1;
      break;
    case 'M':
      medial_mode = (medial_mode%3) + 1;
      break;
    case 'z':
      zoom *= 1/zoom_factor;
      break;
    case 'Z':
      zoom *= zoom_factor;
      break;
    case 'i':
      show_vertex_id = !show_vertex_id;
      break;
    case 'a':
      show_axis = !show_axis;
      break;
    case 'n': {
      for (int i = 0; i < medial_meshes.size(); ++i) {
        medial_meshes[i].InvertOrientation();
      }
      break;
    }
    case 'l':
      show_vertex_distance_lines = !show_vertex_distance_lines;
      break;
    case 'D':
      show_all_vertex_distance_lines = !show_all_vertex_distance_lines;
      break;
    case 'v':
      show_vertices = !show_vertices;
      break;
    case 'u':
      o.simple_q = !o.simple_q;
      GenerateSurface(o);
      break;
    case 'f':
      if (o.max_level < oct::Constants::kMaxLevel) {
        ++o.max_level;
      }
      cout << "Max octree level = " << static_cast<int>(o.max_level) << endl;
      GenerateSurface(o);
      break;
    case 'd':
      o.max_level = max(o.max_level-1, 0);
      cout << "Max octree level = " << static_cast<int>(o.max_level) << endl;
      GenerateSurface(o);
      break;
    case 's':
      o.medial_subdivide_max++;
      cout << "Max medial subdivision level = "
           << o.medial_subdivide_max << endl;
      GenerateSurface(o);
      break;
    case 'S':
      o.medial_subdivide_max--;
      cout << "Max medial subdivision level = "
           << o.medial_subdivide_max << endl;
      GenerateSurface(o);
      break;
    case 'V':
      o.full_subdivide = !o.full_subdivide;
      GenerateSurface(o);
      break;
    case 'B':
      o.make_buffer = !o.make_buffer;
      GenerateSurface(o);
      break;
    case 'e':
      _explode_mode = (_explode_mode+1)%5;
      cout << "Explode mode = " << _explode_mode << endl;
      break;
    case 'E':
      _explode_dir = (_explode_dir+1)%2;
      break;
    case 'J':
      _path_size *= 1.1;
      cout << "Path size = " << _path_size << endl;
      break;
    case 'K':
      _path_size *= 0.9;
      cout << "Path size = " << _path_size << endl;
      break;
    case 'w': {
      cout << "Writing medial mesh to file medial.obj" << endl;
      for (int i = 0; i < medial_meshes.size(); ++i) {
        stringstream ss;
        ss << "medial-" << (i+1) << ".obj";
        ofstream out;
        out.open(ss.str().c_str());
        WriteObj(out, medial_meshes[i]); 
      }

      // ofstream dout("medial.cur");
      // WriteDensity(dout, medial_mesh); 
      cout << "File written" << endl;
      break;
    }
    case 'p':
      show_statistics = !show_statistics;
      break;
    case 'R':
      ResetCenter();
      cout << "Center reset" << endl;
      break;
    case 'r':
      ReadTransformations();
      cout << "View reset" << endl;
      break;
    case 't':
      SaveTransformations();
      cout << "Saved current view" << endl;
      break;
    case 'P':
       writePngImage("sceenshot.png", window_width, window_height);
       break;
    default:
      redisplay = false;
      break;

  }

  if (redisplay) {
    glutPostRedisplay();
  }
}

float Medial3::GetExplode() {
  return _exploded_factor;
}
void Medial3::SetExplode(float f) {
  _exploded_factor = f;
}

void Medial3::IncExplode(float f) {
  _exploded_factor *= f;
}

void Medial3::DecExplode() {
  _exploded_factor *= 0.9;
}

void Medial3::Special(unsigned char key, int x, int y) {
  if (key == GLUT_KEY_LEFT) {
    _target_object = (_target_object+meshes.size()-1) % meshes.size();
    cout << "target object = " << _target_object << endl;
    for (int i = 0; i < 4; ++i) {
      cout << "  " << i << "-ring: "
         << gvd_graph->Ring(_target_object, i).size() << endl;
    }
  } else if (key == GLUT_KEY_RIGHT) {
    _target_object = (_target_object+1) % meshes.size();
    for (int i = 0; i < 4; ++i) {
      cout << "  " << i << "-ring: "
         << gvd_graph->Ring(_target_object, i).size() << endl;
    }
  } else if (key == GLUT_KEY_DOWN) {
    DecExplode();
  } else if (key == GLUT_KEY_UP) {
    IncExplode();
  }
  glutPostRedisplay();
}
