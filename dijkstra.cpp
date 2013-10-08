#include "./dijkstra.h"

#include <set>

const double Dijkstra::MAX = std::numeric_limits<double>::max();

Dijkstra::Dijkstra(const Image& image) : _image(image) {
  const int n = _image.Size();
  _parent.resize(n, -1);
  _cost.resize(n, MAX);
}

struct QNode {
  QNode(int id_, double cost_) : id(id_), cost(cost_) {}
  // static QNode Create(int id, int dx, int dy, double id_cost,
  //                     const Image& image) {
  //   const int nbr = image.Neighbor(id, dx, dy);
  //   return QNode(id, id_cost + image.Cost(nbr, id));
  // }
  // static QNode Create(int id, int nbr, double id_cost,
  //                     const Image& image) {
  //   return QNode(id, id_cost + image.Cost(nbr, id));
  // }
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

void Dijkstra::Run(const int seed) {
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
      // cout << "Processing " << _image.Coords(cur) << endl;

      nbrs.clear();
      _image.Neighbors(cur, back_inserter(nbrs));
      for (int i = 0; i < nbrs.size(); ++i) {
        const int nbr = nbrs[i];
        assert(nbr != cur);
        const int c = _cost[cur] + _image.Cost(cur, nbr);
        if (!visited[nbr] && _cost[nbr] > c) {
          // cout << "  Adding neighbor " << _image.Coords(nbr) << endl;
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

void Dijkstra::Run(const int seedx, const int seedy) {
  Run(_image.Index(seedx, seedy));
}

