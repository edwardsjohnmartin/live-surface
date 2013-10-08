// this is a change.

#include "./snode.h"

int SNode::next_id = 0;
const int SNode::opposite[] = { 1, 0, 3, 2 };
// Given a neighbor direction, gives the lower/upper axis
const int SNode::dir_axis[] = { 1, 1, 0, 0 };
// Given a neighbor direction, what is the index of the lower/upper node?
const int SNode::lower_nodes[] = { 0, 1, 0, 2 };
const int SNode::upper_nodes[] = { 2, 3, 1, 3 };

SNode::SNode() {}
SNode::SNode(shared_ptr<Image> image, const Vec2i& pos, const Vec2i& size) 
    : _image(image), _pos(pos), _size(size), _children(0) {
  _id = next_id++;
  _costBase = _image->MinCost(_pos[0], _pos[1], _size[0], _size[1]).second;
  std::fill(_num_nbrs, _num_nbrs+4, 0);
}

// SNode(shared_ptr<Image> image, const Vec2i& pos, const Vec2i& size,
//       const std::vector<SNode*> nbrs[]) 
//     : _image(image), _pos(pos), _size(size), _children(0) {
//   _costBase = _image->MinCost(_pos[0], _pos[1], _size[0], _size[1]).second;
//   for (int i = 0; i < 4; ++i) {
//     _nbrs[i] = nbrs[i];
//   }
// }

SNode::~SNode() {
  if (_children) delete [] _children;
}

// Case 1
void SNode::UpdateNeighborsCase1(const int dir, SNode* lower, SNode* upper) {
  const int odir = opposite[dir];
  const int axis = dir_axis[dir];
#ifndef NDEBUG
  // these checks are highly inefficient.
  assert(_nbrs[dir].size() > 1);
  for (Nbr_iter it = _nbrs[dir].begin(); it != _nbrs[dir].end(); ++it) {
    assert((*it)->_nbrs[odir].size() == 1);
  }
  assert(lower->_nbrs[dir].empty());
  assert(upper->_nbrs[dir].empty());
#endif

  Nbr_iter it = _nbrs[dir].begin();
  while ((*it)->Center()[axis] < Center()[axis]) {
    // Iterate through neighbors belonging to lower
    lower->_nbrs[dir].push_back(*it);
    (*it)->_nbrs[odir].clear();
    (*it)->_nbrs[odir].push_back(lower);
    ++it;
  } while (it != _nbrs[dir].end()) {
    // Iterate through neighbors belonging to upper
    upper->_nbrs[dir].push_back(*it);
    (*it)->_nbrs[odir].clear();
    (*it)->_nbrs[odir].push_back(upper);
    ++it;
  }

  // Remove neighbors from this
  _nbrs[dir].clear();
}

// Case 2
void SNode::UpdateNeighborsCase2(const int dir, SNode* lower, SNode* upper) {
  const int odir = opposite[dir];
  const int axis = dir_axis[dir];
#ifndef NDEBUG
  // these checks are highly inefficient.
  assert(_nbrs[dir].size() == 1);
  for (Nbr_iter it = _nbrs[dir].begin(); it != _nbrs[dir].end(); ++it) {
    assert(!(*it)->_nbrs[odir].empty());
  }
  assert(lower->_nbrs[dir].empty());
  assert(upper->_nbrs[dir].empty());
#endif

  SNode* nbr = *_nbrs[dir].begin();

  // Add neighbor to lower and upper
  lower->_nbrs[dir].push_back(nbr);
  upper->_nbrs[dir].push_back(nbr);

  // Find where the neighbors points to this
  Nbr_iter it = nbr->_nbrs[odir].begin();
  while ((*it) != this) {
    ++it;
  }
  assert(*it == this);
  // Replace this with lower and upper
  Nbr_iter pos = nbr->_nbrs[odir].erase(it);
  pos = nbr->_nbrs[odir].insert(pos, upper);
  nbr->_nbrs[odir].insert(pos, lower);

  // Remove neighbors from this
  _nbrs[dir].clear();
}


// // void SNode::SetNeighbors(const int dir, const std::vector<SNode*> nbrs) {
// void SNode::SetNeighbors(const int dir, const std::list<SNode*> nbrs) {
//   _nbrs[dir] = nbrs;
// }

// void SNode::SetNeighbor(const int dir, SNode* nbr) {
//   _nbrs[dir].clear();
//   _nbrs[dir].push_back(nbr);
// }

// void SNode::AddNeighbor(const int dir, SNode* nbr) {
//   _nbrs[dir].push_back(nbr);
// }

// void SNode::SetNeighborReflective(const int dir, SNode* nbr) {
//   SetNeighbor(dir, nbr);
//   nbr->AddNeighbor(opposite[dir], this);
// }

// // For use when a new node's edge borders the original edge of the parent
// // node.
// void SNode::SetNeighbors(
//     // const int dir, std::vector<SNode*> nbrs, bool firstHalf) {
//     const int dir, std::list<SNode*> nbrs, bool firstHalf) {
//   assert(_nbrs[dir].empty());

//   // if (nbrs.size() > 1) {
//   //   // The parent has more than one neighbor in this direction
//   //   if (firstHalf) {
//   //     SetNeighborsReflective(dir, nbrs.begin(), nbrs.begin()+nbrs.size()/2);
//   //   } else {
//   //     SetNeighborsReflective(dir, nbrs.begin()+nbrs.size()/2, nbrs.end());
//   //   }
//   // } else if (nbrs.size() == 1) {
//   //   // The parent has one neighbor in this direction
//   //   SetNeighborReflective(dir, nbrs[0]);
//   // }
// }

void SNode::Expand() {
  assert(!_children);
  _children = new SNode[4];
  const int x = _pos[0];
  const int y = _pos[1];
  // const double w = _size[0]/2.0;
  // const double h = _size[1]/2.0;
  const int w = _size[0]/2.0;
  const int h = _size[1]/2.0;
  _children[0] = SNode(_image, Vec2i(x, y), Vec2i(w, h));
  _children[1] = SNode(_image, Vec2i(x+w, y), Vec2i(_size[0]-w, h));
  _children[2] = SNode(_image, Vec2i(x, y+h), Vec2i(w, _size[1]-h));
  _children[3] = SNode(_image, Vec2i(x+w, y+h), Vec2i(_size[0]-w, _size[1]-h));

  // Set internal neighbors
  _children[0]._nbrs[1].push_back(&_children[1]);
  _children[1]._nbrs[0].push_back(&_children[0]);

  _children[1]._nbrs[3].push_back(&_children[3]);
  _children[3]._nbrs[2].push_back(&_children[1]);

  _children[3]._nbrs[0].push_back(&_children[2]);
  _children[2]._nbrs[1].push_back(&_children[3]);

  _children[2]._nbrs[2].push_back(&_children[0]);
  _children[0]._nbrs[3].push_back(&_children[2]);

  // Update external neighbors
  for (int dir = 0; dir < 4; ++dir) {
    SNode* lower = &_children[lower_nodes[dir]];
    SNode* upper = &_children[upper_nodes[dir]];
    if (_nbrs[dir].empty()) {
      // do nothing
    } else if (*_nbrs[dir].begin() == *_nbrs[dir].rbegin()) {
      // one element: case 2
      UpdateNeighborsCase2(dir, lower, upper);
    } else {
      // more than one element: case 1
      UpdateNeighborsCase1(dir, lower, upper);
    }
  }

  // // top-left (0)
  // //   -x neighbors: first-half of this -x neighbors
  // //   +x neighbors: _children[1]
  // //   -y neighbors: first-half of this -y neighbors
  // //   +y neighbors: _children[2]
  // _children[0].SetNeighbors(0, _nbrs[0], true);
  // _children[0].SetNeighbor(1, &_children[1]);
  // _children[0].SetNeighbors(2, _nbrs[2], true);
  // _children[0].SetNeighbor(3, &_children[2]);

  // // top-right (1)
  // //   -x neighbors: _children[0]
  // //   +x neighbors: first-half of this +x neighbors
  // //   -y neighbors: second-half of this -y neighbors
  // //   +y neighbors: _children[3]
  // _children[1].SetNeighbor(0, &_children[0]);
  // _children[1].SetNeighbors(1, _nbrs[1], true);
  // _children[1].SetNeighbors(2, _nbrs[2], false);
  // _children[1].SetNeighbor(3, &_children[3]);

  // // bottom-left (2)
  // //   -x neighbors: second-half of this -x neighbors
  // //   +x neighbors: _children[3]
  // //   -y neighbors: _children[0]
  // //   +y neighbors: first-half of this +y neighbors
  // _children[2].SetNeighbors(0, _nbrs[0], false);
  // _children[2].SetNeighbor(1, &_children[3]);
  // _children[2].SetNeighbor(2, &_children[0]);
  // _children[2].SetNeighbors(3, _nbrs[3], true);

  // // bottom-right (3)
  // //   -x neighbors: _children[2]
  // //   +x neighbors: second-half of this +x neighbors
  // //   -y neighbors: _children[1]
  // //   +y neighbors: second-half of this +y neighbors
  // _children[3].SetNeighbor(0, &_children[2]);
  // _children[3].SetNeighbors(1, _nbrs[1], false);
  // _children[3].SetNeighbor(2, &_children[1]);
  // _children[3].SetNeighbors(3, _nbrs[3], false);

  // for (int i = 0; i < 4; ++i) {
  //   _nbrs[i].clear();
  // }
}

