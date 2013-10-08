#include "./dijkstra2.h"

#include <set>

#include "./snode.h"

struct QNode {
  QNode(int id_, double cost_) : id(id_), cost(cost_) {}
  int id;
  double cost;
  bool operator<(const QNode& rhs) const {
    return cost < rhs.cost;
  }
};

std::ostream& operator<<(std::ostream& out,
                        const std::pair<int, int>& p) {
  out << "(" << p.first << ", " << p.second << ")";
  return out;
}

const double Dijkstra2::MAX = std::numeric_limits<double>::max();

// Dijkstra2::Dijkstra2(const ImageGraph& image) : _image(image) {
//   const int n = _image.Size();
//   _parent.resize(n, -1);
//   _cost.resize(n, MAX);
// }

Dijkstra2::Dijkstra2(const Image& image) : _image(ImageGraph(image)) {
// Dijkstra2::Dijkstra2(const Image& image, const SNode& sroot) {
  

  const int n = _image.Size();
  _parent.resize(n, -1);
  _cost.resize(n, MAX);
}

void Dijkstra2::Run(const int seed) {
  using namespace std;

  const int n = _image.Size();
  _parent.resize(n, -1);
  _cost.resize(n, MAX);
  vector<bool> visited(n, false);

  _parent[seed] = seed;
  _cost[seed] = 0;
  multiset<QNode> queue;
  queue.insert(QNode(seed, 0));
  vector<int> nbrs;
  nbrs.reserve(27);
  do {
    QNode n = *queue.begin();
    const int cur = n.id;
    queue.erase(queue.begin());
    if (!visited[cur]) {
      visited[cur] = true;

      nbrs.clear();
      _image.Neighbors(cur, back_inserter(nbrs));
      for (int i = 0; i < nbrs.size(); ++i) {
        const int nbr = nbrs[i];
        assert(nbr != cur);
        const int c = _cost[cur] + _image.Cost(cur, nbr);
        if (!visited[nbr] && _cost[nbr] > c) {
          queue.insert(QNode(nbr, c));
          _parent[nbr] = cur;
          _cost[nbr] = c;
        }
      }
    } else {
      // cout << "Already visited " << _image.Coords(cur) << endl;
    }
  } while (!queue.empty());
}

void Dijkstra2::Run(const int seedx, const int seedy) {
  Run(_image.GraphIndex(seedx, seedy));
}

