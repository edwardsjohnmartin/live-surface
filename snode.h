#ifndef __SNODE_H__
#define __SNODE_H__

#include <list>

#include "./image.h"
#include "./shared_ptr.h"

// Quadrant and neighbor coordinates use image convention of the origin
// being in the top-left corner.

// Quadrants
//  -------------------------
// |           |             |
// |     0     |      1      |
// |           |             |
// |           |             |
//  -------------------------
// |           |             |
// |     2     |      3      |
// |           |             |
// |           |             |
//  -------------------------

// Neighbors
//             ----------
//            |    -y    |
//            |     2    |
//            |          |
//  --------------------------------
// |   -x     |          |   +x     |
// |    0     |          |    1     |
// |          |          |          |
//  --------------------------------
//            |    +y    |
//            |     3    |
//            |          |
//             ----------

// Two cases for subdividing upper-right (index 1) node
//
// Case 1
// Node 1 has 2 0-neighbors that are                      | subdivision
// split between two new nodes (0,2)                      v
//  -------------------------           -------------------------
// |     |     |             |         |     |     |      |      |
// |_____|_____|             |         |_____|_____|______|______|
// |     |     |             |         |     |     |      |      |
// |     |     |             |         |     |     |      |      |
//  -------------------------    -->    -------------------------
// |           |             |         |           |             |
// |           |             |         |           |             |
// |           |             |         |           |             |
// |           |             |         |           |             |
//  -------------------------           -------------------------
//
// Case 2                                             | subdivision
// Node 0 has multiple 1-neighbors.                   v
//  -------------------------           -------------------------
// |           |      |      |         |           |___|__|      |
// |           |______|______|         |           |___|__|______|
// |           |      |      |         |           |      |      |
// |           |      |      |         |           |      |      |
//  -------------------------    -->    -------------------------
// |           |             |         |           |             |
// |           |             |         |           |             |
// |           |             |         |           |             |
// |           |             |         |           |             |
//  -------------------------           -------------------------

// Simplification node - a quadtree-style data structure to
// store how an image is simplified
class SNode {
 public:
  // typedef std::vector<SNode*>::iterator Nbr_iter;
  // typedef std::vector<SNode*>::const_iterator Nbr_const_iter;
  typedef std::list<SNode*>::iterator Nbr_iter;
  typedef std::list<SNode*>::const_iterator Nbr_const_iter;

 public:
  SNode();
  SNode(shared_ptr<Image> image, const Vec2i& pos, const Vec2i& size);

  // SNode(shared_ptr<Image> image, const Vec2i& pos, const Vec2i& size,
  //       const std::vector<SNode*> nbrs[]) 
  //     : _image(image), _pos(pos), _size(size), _children(0) {
  //   _costBase = _image->MinCost(_pos[0], _pos[1], _size[0], _size[1]).second;
  //   for (int i = 0; i < 4; ++i) {
  //     _nbrs[i] = nbrs[i];
  //   }
  // }

  ~SNode();

  // Case 1 (multiple neighbors in direction dir)
  // "this" is the node being subdivided.
  // lower/upper - two newly-created nodes on the dir side of "this"
  void UpdateNeighborsCase1(const int dir, SNode* lower, SNode* upper);
  void UpdateNeighborsCase2(const int dir, SNode* lower, SNode* upper);

  // // void SetNeighbors(const int dir, const std::vector<SNode*> nbrs);
  // void SetNeighbors(const int dir, const std::list<SNode*> nbrs);

  // // Assigns neighbors [begin, end) to this node and assigns this node
  // // as the neighbor, replacing this' parent.
  // template <typename Iter>
  // void SetNeighborsReflective(const int dir, Iter begin, Iter end) {
  //   assert(_nbrs[dir].empty());
  //   _nbrs[dir].insert(_nbrs[dir].begin(), begin, end);

  //   for (Iter it = begin; it != end; ++it) {
  //     (*it)->SetNeighbor(opposite[dir], this);
  //   }
  // }

  // void SetNeighbor(const int dir, SNode* nbr);
  // void AddNeighbor(const int dir, SNode* nbr);
  // void SetNeighborReflective(const int dir, SNode* nbr);

  // // For use when a new node's edge borders the original edge of the parent
  // // node.
  // void SetNeighbors(
  //     // const int dir, std::vector<SNode*> nbrs, bool firstHalf);
  //     const int dir, std::list<SNode*> nbrs, bool firstHalf);

  Nbr_iter NbrsBegin(int dir) { return _nbrs[dir].begin(); }
  Nbr_iter NbrsEnd(int dir) { return _nbrs[dir].end(); }
  Nbr_const_iter NbrsBegin(int dir) const { return _nbrs[dir].begin(); }
  Nbr_const_iter NbrsEnd(int dir) const { return _nbrs[dir].end(); }

  const Vec2i& Pos() const { return _pos; }
  const Vec2i& Size() const { return _size; }
  const Vec2i Center() const {
    return _pos + _size/2;
  }

  double CostBase() const { return _costBase; }
  double Cost(const SNode& n) const {
    return _costBase * (Center()-n.Center()).norm();
  }

  const bool IsExpanded() const { return _children; }

  void Expand();

  SNode& operator[](int i) { return _children[i]; }
  const SNode& operator[](int i) const { return _children[i]; }

  int id() const { return _id; }

 private:
  static int next_id;
  static const int opposite[];
  static const int dir_axis[];
  static const int lower_nodes[];
  static const int upper_nodes[];
  int _id;
  shared_ptr<Image> _image;
  Vec2i _pos;
  Vec2i _size;
  // std::vector<SNode> _children;
  SNode* _children;
  // Neighbors in the 4 cardinal directions: -x, +x, -y, +y
  // std::vector<int> _nbrs[4];
  // std::vector<SNode*> _nbrs[4];
  std::list<SNode*> _nbrs[4];
  int _num_nbrs[4];
  // Total cost from this node n to another node m is _costBase*|n-m|
  double _costBase;
};

#endif
