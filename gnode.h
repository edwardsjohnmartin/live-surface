#ifndef __GNODE_H__
#define __GNODE_H__

class GNode {
 public:
  GNode() {}

  // // pos - coordinates of node in image coordinates
  // // size - width/height of node, with original pixels of size 1/1
  // GNode(const Vec2i pos, const Vec2i size,
  //       const std::vector<int>& nbrs, const std::vector<double>& costs)
  //     : _pos(pos), _size(size), _nbrs(nbrs), _costs(costs) {
  //   for (int i = 0; i < nbrs.size(); ++i) {
  //     _nbrGlobal2Local[nbrs[i]] = i;
  //   }
  // }

  // GNode(const std::vector<int>& nbrs, const std::vector<double>& costs)
  //     : /*_nbrs(nbrs),*/ _costs(costs) {
  //   _nbrs[0] = nbrs;
  //   for (int i = 0; i < _nbrs[0].size(); ++i) {
  //     _nbrGlobal2Local[_nbrs[0][i]] = i;
  //   }
  // }

  GNode(const std::vector<int>& nbrs, const double& cost)
      : _cost(cost) {
    _nbrs[0] = nbrs;
    for (int i = 0; i < _nbrs[0].size(); ++i) {
      _nbrGlobal2Local[_nbrs[0][i]] = i;
    }
  }

  int Degree(int dir) const { return _nbrs[dir].size(); }
  // Index of the ith neighbor
  int Nbr(int dir, int i) const { return _nbrs[dir][i]; }
  // Cost from nbr back to this node
  // double Cost(int nbr) const { return _costs[nbr]; }
  // double Cost(int nbrGlobal) const { return _costs[_nbrGlobal2Local[nbrGlobal]]; }
  // double Cost(int nbrGlobal) const { return _costs[_nbrGlobal2Local.find(nbrGlobal)->second]; }
  double Cost(const GNode& nbr) const { return _cost; }

 private:
  // Vec2i _pos;
  // Vec2i _size;
  std::vector<int> _nbrs[4];
  // std::vector<double> _costs;
  double _cost;
  std::map<int, int> _nbrGlobal2Local;
};

#endif
