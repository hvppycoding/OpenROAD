#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include <deque>
#include <map>
#include <vector>
#include <numeric>

#include "odb/geom.h"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"

using namespace std;

namespace ord {
class OpenRoad;
}

namespace odb {
class dbDatabase;
class dbNet;
}  // namespace odb

namespace gui {
class Gui;
}

namespace stt {

using odb::Point;
using stt::Tree;
using utl::Logger;

class UnionFind {
 private:
  std::vector<int> parent, rank;

 public:
  // Constructor
  UnionFind(int size) : parent(size), rank(size, 0) {
    for (int i = 0; i < size; i++) {
      parent[i] = i;  // Initially, every element is its own parent.
    }
  }

  // Find the root of the set in which element 'p' belongs
  int find(int p) {
    while (p != parent[p]) {
      parent[p] = parent[parent[p]];  // Path compression
      p = parent[p];
    }
    return p;
  }

  // Unite the sets containing elements 'p' and 'q'
  void unite(int p, int q) {
    int rootP = find(p);
    int rootQ = find(q);
    if (rootP == rootQ) return;  // They are already in the same set

    // Make the root of the smaller rank point to the root of the larger rank
    if (rank[rootP] < rank[rootQ]) {
      parent[rootP] = rootQ;
    } else if (rank[rootP] > rank[rootQ]) {
      parent[rootQ] = rootP;
    } else {
      parent[rootQ] = rootP;
      rank[rootP]++;
    }
  }

  // Check if the elements 'p' and 'q' are in the same set
  bool connected(int p, int q) { return find(p) == find(q); }
};

class TDPoint {
 public:
  TDPoint() : x_(0), y_(0) {}
  TDPoint(int x, int y) : x_(x), y_(y) {}
  int x_;
  int y_;
};

class TDPin {
 public:
  int x_;
  int y_;
  bool is_driver_;
  double slack_;
  double arrival_time_;
};

class TDNet {
 public:
  const odb::dbNet* dbnet_;
  bool is_clock_;
  int n_pins_;
  std::vector<int> x_;
  std::vector<int> y_;
  int driver_index_;
  std::vector<double> slack_;
  std::vector<double> arrival_time_;
  std::vector<TDPin*> pins_;
};

class TDEdge {
 public:
  TDEdge() : cap(0), usage(0), red(0) {}

  unsigned short cap;
  unsigned short usage;
  unsigned short red;
};

typedef std::pair<int, int> RE;

class RES {
 public:
  RES() {}
  RES(const vector<int>& res) { initialize_from_1d(res); }
  RES(const RES& res) { res_ = res.res_; }
  void initialize_from_1d(const vector<int>& res);
  RE get(int i) const { return res_[i]; }
  int count() const { return res_.size(); }
  void add(int i, int j) { res_.push_back(std::make_pair(i, j)); }
  void add(const RE& re) { res_.push_back(re); }
  void erase(int i) { res_.erase(res_.begin() + i, res_.begin() + i + 1); }

  std::vector<RE>::iterator begin() { return res_.begin(); }
  std::vector<RE>::iterator end() { return res_.end(); }
  std::vector<RE>::const_iterator begin() const { return res_.begin(); }
  std::vector<RE>::const_iterator end() const { return res_.end(); }

 private:
  vector<RE> res_;
};

class RESTree {
 public:
  RESTree(TDNet* net, const RES& res) {
    net_ = net;
    res_ = res;
    pins_ = net->pins_;
    n_pins_ = pins_.size();
    driver_index_ = -1;
    for (int i = 0; i < pins_.size(); i++) {
      if (pins_[i]->is_driver_) {
        assert(driver_index_ == -1);
        driver_index_ = i;
      }
    }
    initialize();
  }

  void updateRES(const RES& res) {
    res_ = res;
    initialize();
  }

  void initialize() {
    x_.resize(n_pins_);
    y_.resize(n_pins_);
    x_low_.resize(n_pins_);
    x_high_.resize(n_pins_);
    y_low_.resize(n_pins_);
    y_high_.resize(n_pins_);

    for (int i = 0; i < pins_.size(); i++) {
      x_[i] = pins_[i]->x_;
      y_[i] = pins_[i]->y_;
      x_low_[i] = x_[i];
      x_high_[i] = x_[i];
      y_low_[i] = y_[i];
      y_high_[i] = y_[i];
    }
    for (int i = 0; i < res_.count(); i++) {
      RE re = res_.get(i);
      int nv = re.first;
      int nh = re.second;
      for (int i = 0; i < pins_.size(); i++) {
        x_low_[nh] = std::min(x_low_[nh], x_[nv]);
        x_high_[nh] = std::max(x_high_[nh], x_[nv]);
        y_low_[nv] = std::min(y_low_[nv], y_[nh]);
        y_high_[nv] = std::max(y_high_[nv], y_[nh]);
      }
    }

    length_ = 0;
    for (int i = 0; i < n_pins_; i++) {
      length_ += x_high_[i] - x_low_[i] + y_high_[i] - y_low_[i];
    }
  }

  int numPins() const { return n_pins_; }
  TDPin* getPin(int i) const { return pins_[i]; }
  int length() const { return length_; }
  int x(int i) const { return x_[i]; }
  int y(int i) const { return y_[i]; }
  int x_low(int i) const { return x_low_[i]; }
  int x_high(int i) const { return x_high_[i]; }
  int y_low(int i) const { return y_low_[i]; }
  int y_high(int i) const { return y_high_[i]; }
  int driverIndex() const { return driver_index_; }
  string toString() const {
    string s = "RESTree\n";
    for (int i = 0; i < n_pins_; i++) {
      s += "Pin " + std::to_string(i) + " (" + std::to_string(x_[i]) + ", " +
           std::to_string(y_[i]) + ")";
      if (i == driver_index_) {
        s += " (driver)";
      }
      s += "\n";
    }
    for (int i = 0; i < res_.count(); i++) {
      RE re = res_.get(i);
      s += "Edge " + std::to_string(re.first) + " -> " +
           std::to_string(re.second) + "\n";
    }

    return s;
  }

  const RES& getRES() const { return res_; }

 private:
  TDNet* net_;
  RES res_;
  int n_pins_;
  int driver_index_;
  int length_;
  vector<TDPin*> pins_;
  vector<int> x_;
  vector<int> y_;
  vector<int> x_low_;
  vector<int> x_high_;
  vector<int> y_low_;
  vector<int> y_high_;
};


class OverflowManager {
 public:
  OverflowManager() {

  }

  void init(int x_grid, int y_grid) {
    x_grid_ = x_grid;
    y_grid_ = y_grid;
    hcapacity_map_.resize(y_grid_);
    vcapacity_map_.resize(y_grid_);
    husage_map_.resize(y_grid_);
    vusage_map_.resize(y_grid_);
    for (int y = 0; y < y_grid_; y++) {
      hcapacity_map_[y].resize(x_grid_, 0);
      vcapacity_map_[y].resize(x_grid_, 0);
      husage_map_[y].resize(x_grid_, 0);
      vusage_map_[y].resize(x_grid_, 0);
    }
  }

  void setVCapacity(int x, int y, int cap) { vcapacity_map_[y][x] = cap; }
  void setHCapacity(int x, int y, int cap) { hcapacity_map_[y][x] = cap; }
  int countHOverflow(int y, int xl, int xr) {
    int overflow = 0;
    for (int x = xl; x < xr; x++) {
      if (husage_map_[y][x] > hcapacity_map_[y][x]) {
        overflow += 1;
      }
    }
    return overflow;
  }

  int countVOverflow(int x, int yl, int yr) {
    int overflow = 0;
    for (int y = yl; y < yr; y++) {
      if (vusage_map_[y][x] > vcapacity_map_[y][x]) {
        overflow += 1;
      }
    }
    return overflow;
  }

  void changeVUsage(int x, int yl, int yh, int delta = 1) {
    for (int y = yl; y < yh; y++) {
      vusage_map_[y][x] += delta;
    }
  }

  void changeHUsage(int y, int xl, int xr, int delta = 1) {
    for (int x = xl; x < xr; x++) {
      husage_map_[y][x] += delta;
    }
  }

  static OverflowManager* createRandom(int nx, int ny) {
    OverflowManager* overflow_manager = new OverflowManager();
    overflow_manager->init(nx, ny);
    for (int y = 0; y < ny; y++) {
      for (int x = 0; x < nx; x++) {
        overflow_manager->setVCapacity(x, y, rand() % 10);
        overflow_manager->changeVUsage(x, y, y + 1, rand() % 10);
        overflow_manager->setHCapacity(x, y, rand() % 10);
        overflow_manager->changeHUsage(y, x, x + 1, rand() % 10);
      }
    }
    return overflow_manager;
  }

  void addUsage(const RESTree* tree) {
    for (int i = 0; i < tree->numPins(); i++) {
      int y = tree->y(i);
      int x_low = tree->x_low(i);
      int x_high = tree->x_high(i);
      changeHUsage(y, x_low, x_high);
    }

    for (int i = 0; i < tree->numPins(); i++) {
      int x = tree->x(i);
      int y_low = tree->y_low(i);
      int y_high = tree->y_high(i);
      changeVUsage(x, y_low, y_high);
    }
  }

  void removeUsage(const RESTree* tree) {
    for (int i = 0; i < tree->numPins(); i++) {
      int y = tree->y(i);
      int x_low = tree->x_low(i);
      int x_high = tree->x_high(i);
      changeHUsage(y, x_low, x_high, -1);
    }

    for (int i = 0; i < tree->numPins(); i++) {
      int x = tree->x(i);
      int y_low = tree->y_low(i);
      int y_high = tree->y_high(i);
      changeVUsage(x, y_low, y_high, -1);
    }
  }

 private:
  int x_grid_;
  int y_grid_;
  vector<vector<int>> hcapacity_map_;
  vector<vector<int>> vcapacity_map_;
  vector<vector<int>> husage_map_;
  vector<vector<int>> vusage_map_;
};

class RESTreeAbstractEvaluator {
 public:
  RESTreeAbstractEvaluator() { weight_ = 1.0; }
  virtual double getCost(RESTree* tree) = 0;

  double weight() const { return weight_; }
  void setWeight(double weight) { weight_ = weight; }

 private:
  double weight_;
};

class RESTreeLengthEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getCost(RESTree* tree) { return tree->length() * weight(); }
};

class RESTreeOverflowEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeOverflowEvaluator(OverflowManager* overflow_manager) {
    overflow_manager_ = overflow_manager;
  }
  double getCost(RESTree* tree) {
    int overflow = 0;

    for (int i = 0; i < tree->numPins(); i++) {
      int y = tree->y(i);
      int x_low = tree->x_low(i);
      int x_high = tree->x_high(i);
      overflow += overflow_manager_->countHOverflow(y, x_low, x_high);
    }

    for (int i = 0; i < tree->numPins(); i++) {
      int x = tree->x(i);
      int y_low = tree->y_low(i);
      int y_high = tree->y_high(i);
      overflow += overflow_manager_->countVOverflow(x, y_low, y_high);
    }
    return overflow * weight();
  }

 private:
  OverflowManager* overflow_manager_;
};

class ConverterNode {
 public:
  ConverterNode(int index, int x, int y, bool is_pin, TDPin* pin = nullptr) {
    index_ = index;
    x_ = x;
    y_ = y;
    is_pin_ = is_pin;
    pin_ = pin;
  }

  void removeNeighbor(const ConverterNode* neighbor) {
    right_neighbors_.erase(
        std::remove(right_neighbors_.begin(), right_neighbors_.end(), neighbor),
        right_neighbors_.end());
    left_neighbors_.erase(
        std::remove(left_neighbors_.begin(), left_neighbors_.end(), neighbor),
        left_neighbors_.end());
    up_neighbors_.erase(
        std::remove(up_neighbors_.begin(), up_neighbors_.end(), neighbor),
        up_neighbors_.end());
    down_neighbors_.erase(
        std::remove(down_neighbors_.begin(), down_neighbors_.end(), neighbor),
        down_neighbors_.end());
  }

  TDPin* getPin() const {
    if (is_pin_) {
      return pin_;
    } else {
      return nullptr;
    }
  }

  void addHorizontalNeighbor(ConverterNode* neighbor) {
    if (neighbor->x_ <= x_) {
      addLeftNeighbor(neighbor);
    } else {
      addRightNeighbor(neighbor);
    }
  }

  void addVerticalNeighbor(ConverterNode* neighbor) {
    if (neighbor->y_ <= y_) {
      addDownNeighbor(neighbor);
    } else {
      addUpNeighbor(neighbor);
    }
  }

  void addLeftNeighbor(ConverterNode* neighbor) {
    assert(neighbor->x_ <= x_);
    assert(left_neighbors_.find(neighbor) == left_neighbors_.end());
    for (int i = left_neighbors_.size() - 1; i >= 0; i--) {
      if (left_neighbors_[i]->x_ >= neighbor->x_) {
        left_neighbors_.insert(left_neighbors_.begin() + i + 1, neighbor);
        return;
      }
    }
    left_neighbors_.insert(left_neighbors_.begin(), neighbor);
  }

  void addRightNeighbor(ConverterNode* neighbor) {
    assert(neighbor->x_ >= x_);
    assert(right_neighbors_.find(neighbor) == right_neighbors_.end());
    for (int i = right_neighbors_.size() - 1; i >= 0; i--) {
      if (right_neighbors_[i]->x_ <= neighbor->x_) {
        right_neighbors_.insert(right_neighbors_.begin() + i + 1, neighbor);
        return;
      }
    }
    right_neighbors_.insert(right_neighbors_.begin(), neighbor);
  }

  void addUpNeighbor(ConverterNode* neighbor) {
    assert(neighbor->y_ >= y_);
    assert(up_neighbors_.find(neighbor) == up_neighbors_.end());
    for (int i = up_neighbors_.size() - 1; i >= 0; i--) {
      if (up_neighbors_[i]->y_ <= neighbor->y_) {
        up_neighbors_.insert(up_neighbors_.begin() + i + 1, neighbor);
        return;
      }
    }
    up_neighbors_.insert(up_neighbors_.begin(), neighbor);
  }

  void addDownNeighbor(ConverterNode* neighbor) {
    assert(neighbor->y_ <= y_);
    assert(down_neighbors_.find(neighbor) == down_neighbors_.end());
    for (int i = down_neighbors_.size() - 1; i >= 0; i--) {
      if (down_neighbors_[i]->y_ >= neighbor->y_) {
        down_neighbors_.insert(down_neighbors_.begin() + i + 1, neighbor);
        return;
      }
    }
    down_neighbors_.insert(down_neighbors_.begin(), neighbor);
  }

  const vector<ConverterNode*>& getRightNeighbors() const {
    return right_neighbors_;
  }

  const vector<ConverterNode*>& getLeftNeighbors() const {
    return left_neighbors_;
  }

  const vector<ConverterNode*>& getUpNeighbors() const { return up_neighbors_; }

  const vector<ConverterNode*>& getDownNeighbors() const {
    return down_neighbors_;
  }

  int index() const { return index_; }
  int x() const { return x_; }
  int y() const { return y_; }
  bool isPin() const { return is_pin_; }
  TDPin* pin() const { return pin_; }

 private:
  int index_;
  int x_;
  int y_;
  bool is_pin_;
  TDPin* pin_;
  vector<ConverterNode*> right_neighbors_;
  vector<ConverterNode*> left_neighbors_;
  vector<ConverterNode*> up_neighbors_;
  vector<ConverterNode*> down_neighbors_;
};

enum class EdgeType { H2V_RIGHT, H2V_LEFT, V2H_UP, V2H_DOWN, UNKNOWN };

class SteinerCandidate {
 public:
  SteinerCandidate(int gain, int x, int y, ConverterNode* node,
                   ConverterNode* op_node1, ConverterNode* op_node2,
                   EdgeType edge_type) {
    update(gain, x, y, node, op_node1, op_node2, edge_type);
  }

  void update(int gain, int x, int y, ConverterNode* node,
              ConverterNode* op_node1, ConverterNode* op_node2,
              EdgeType edge_type) {
    gain_ = gain;
    x_ = x;
    y_ = y;
    node_ = node;
    op_node1_ = op_node1;
    op_node2_ = op_node2;
    edge_type_ = edge_type;
  }

  int gain_;
  int x_;
  int y_;
  ConverterNode* node_;
  ConverterNode* op_node1_;
  ConverterNode* op_node2_;
  EdgeType edge_type_;
};

class CmpSteinerCandidate {
 public:
  bool operator()(const SteinerCandidate& a, const SteinerCandidate& b) const {
    return a.gain_ < b.gain_;
  }
};

class SteinerNode {
 public:
  SteinerNode(int index, int x, int y, bool is_pin, TDPin* pin = nullptr) {
    index_ = index;
    x_ = x;
    y_ = y;
    is_pin_ = is_pin;
    pin_ = pin;
  }

  int index() const { return index_; }
  int x() const { return x_; }
  int y() const { return y_; }
  void addNeighbor(SteinerNode* neighbor) { neighbors_.push_back(neighbor); }
  void removeNeighbor(SteinerNode* neighbor) {
    neighbors_.erase(
        std::remove(neighbors_.begin(), neighbors_.end(), neighbor),
        neighbors_.end());
  }
  SteinerNode* getNeighbor(int i) const { return neighbors_[i]; }
  vector<SteinerNode*> getNeighbors() const { return neighbors_; }
  bool isNeighbor(SteinerNode* neighbor) const {
    return std::find(neighbors_.begin(), neighbors_.end(), neighbor) !=
           neighbors_.end();
  }
  int countNeighbors() const { return neighbors_.size(); }
  void clearNeighbors() { neighbors_.clear(); }

  int index_;
  int x_;
  int y_;
  bool is_pin_;
  TDPin* pin_;
  vector<SteinerNode*> neighbors_;
};

class SteinerGraph {
 public:
  SteinerGraph(vector<SteinerNode*> nodes) : nodes_(nodes) {}
  ~SteinerGraph() {
    for (SteinerNode* node : nodes_) {
      delete node;
    }
  }

  vector<SteinerNode*> nodes() const { return nodes_; }
  SteinerNode* getNode(int i) { return nodes_[i]; }
  int nodeCount() const { return nodes_.size(); }
  void addNode(SteinerNode* node) { nodes_.push_back(node); }

  string toString() const {
    string s = "SteinerGraph\n";
    for (SteinerNode* node : nodes_) {
      s += "Node " + std::to_string(node->index()) + " (" +
           std::to_string(node->x()) + ", " + std::to_string(node->y()) + ")";
      if (node->is_pin_) {
        s += " (pin)";
      }
      s += "\n";
      for (SteinerNode* neighbor : node->getNeighbors()) {
        s += "  Neighbor " + std::to_string(neighbor->index()) + " (" +
             std::to_string(neighbor->x()) + ", " +
             std::to_string(neighbor->y()) + ")\n";
      }
    }
    return s;
  }

 private:
  vector<SteinerNode*> nodes_;
};

class TreeConverter {
 public:
  TreeConverter(RESTree* tree) : restree_(tree) {}

  void initializeNodes() {
    nodes_.clear();
    for (int i = 0; i < restree_->numPins(); i++) {
      TDPin* pin = restree_->getPin(i);
      ConverterNode* node = new ConverterNode(i, pin->x_, pin->y_, true, pin);
      nodes_.push_back(node);
    }
  }

  void initializeEdges() {
    for (auto [nv, nh] : restree_->getRES()) {
      ConverterNode* node_v = nodes_[nv];
      ConverterNode* node_h = nodes_[nh];
      node_v->addVerticalNeighbor(node_h);
      node_h->addHorizontalNeighbor(node_v);
    }
  }

  SteinerCandidate bestSteinerForNode(ConverterNode* node) {
    SteinerCandidate best(/* gain= */ 0,
                          /* x= */ 0,
                          /* y= */ 0,
                          /* node= */ nullptr,
                          /* op1= */ nullptr,
                          /* op2= */ nullptr,
                          /* edge_type= */ EdgeType::UNKNOWN);
    int gain = 0;
    ConverterNode* op1 = nullptr;
    ConverterNode* op2 = nullptr;

    if (node->getRightNeighbors().size() >= 2) {
      int n = node->getRightNeighbors().size();
      op1 = node->getRightNeighbors()[n - 1];
      op2 = node->getRightNeighbors()[n - 2];
      gain = op2->x() - node->x();
      if (gain > best.gain_) {
        best.update(gain, op2->x(), node->y(), node, op1, op2,
                    EdgeType::H2V_RIGHT);
      }
    }

    if (node->getLeftNeighbors().size() >= 2) {
      int n = node->getLeftNeighbors().size();
      op1 = node->getLeftNeighbors()[n - 1];
      op2 = node->getLeftNeighbors()[n - 2];
      gain = node->x() - op2->x();
      if (gain > best.gain_) {
        best.update(gain, op2->x(), node->y(), node, op1, op2,
                    EdgeType::H2V_LEFT);
      }
    }

    if (node->getUpNeighbors().size() >= 2) {
      int n = node->getUpNeighbors().size();
      op1 = node->getUpNeighbors()[n - 1];
      op2 = node->getUpNeighbors()[n - 2];
      gain = op2->y() - node->y();
      if (gain > best.gain_) {
        best.update(gain, node->x(), op2->y(), node, op1, op2,
                    EdgeType::V2H_UP);
      }
    }

    if (node->getDownNeighbors().size() >= 2) {
      int n = node->getDownNeighbors().size();
      op1 = node->getDownNeighbors()[n - 1];
      op2 = node->getDownNeighbors()[n - 2];
      gain = node->y() - op2->y();
      if (gain > best.gain_) {
        best.update(gain, node->x(), op2->y(), node, op1, op2,
                    EdgeType::V2H_DOWN);
      }
    }

    return best;
  }

  void steinerize() {
    using Heap =
        boost::heap::d_ary_heap<SteinerCandidate, boost::heap::mutable_<true>,
                                boost::heap::arity<2>,
                                boost::heap::compare<CmpSteinerCandidate>>;
    Heap heap;
    std::map<ConverterNode*, Heap::handle_type> handles;

    for (ConverterNode* node : nodes_) {
      handles[node] = heap.push(bestSteinerForNode(node));
    }

    while (!heap.empty()) {
      const SteinerCandidate best = heap.top();
      heap.pop();

      if (best.gain_ <= 0) {
        break;
      }

      ConverterNode* node = best.node_;
      ConverterNode* op1 = best.op_node1_;
      ConverterNode* op2 = best.op_node2_;

      bool new_node = false;
      ConverterNode* steiner_node = nullptr;

      if (best.x_ == op2->x() && best.y_ == op2->y()) {
        steiner_node = op2;
      } else if (best.x_ == op1->x() && best.y_ == op1->y()) {
        steiner_node = op1;
      } else {
        steiner_node =
            new ConverterNode(nodes_.size(), best.x_, best.y_, false);
        nodes_.push_back(steiner_node);
        new_node = true;
      }

      if (best.edge_type_ == EdgeType::V2H_UP ||
          best.edge_type_ == EdgeType::V2H_DOWN) {
        if (steiner_node != op1) {
          op1->removeNeighbor(node);
          op1->addHorizontalNeighbor(steiner_node);
          steiner_node->addVerticalNeighbor(op1);
          node->removeNeighbor(op1);
        }
        if (steiner_node != op2) {
          op2->removeNeighbor(node);
          op2->addHorizontalNeighbor(steiner_node);
          steiner_node->addVerticalNeighbor(op2);
          node->removeNeighbor(op2);
        }
        if (new_node) {
          node->addVerticalNeighbor(steiner_node);
          steiner_node->addHorizontalNeighbor(node);
          SteinerCandidate new_best_steiner = bestSteinerForNode(steiner_node);
          handles[steiner_node] = heap.push(new_best_steiner);
        }
      } else {
        if (steiner_node != op1) {
          op1->removeNeighbor(node);
          op1->addVerticalNeighbor(steiner_node);
          steiner_node->addHorizontalNeighbor(op1);
          node->removeNeighbor(op1);
        }
        if (steiner_node != op2) {
          op2->removeNeighbor(node);
          op2->addVerticalNeighbor(steiner_node);
          steiner_node->addHorizontalNeighbor(op2);
          node->removeNeighbor(op2);
        }
        if (new_node) {
          node->addHorizontalNeighbor(steiner_node);
          steiner_node->addVerticalNeighbor(node);
          SteinerCandidate new_best_steiner = bestSteinerForNode(steiner_node);
          handles[steiner_node] = heap.push(new_best_steiner);
        }
      }

      SteinerCandidate new_best_node = bestSteinerForNode(node);
      handles[node] = heap.push(new_best_node);

      *handles[op1] = bestSteinerForNode(op1);
      heap.update(handles[op1]);
      *handles[op2] = bestSteinerForNode(op2);
      heap.update(handles[op2]);
    }
  }

  SteinerGraph* makeSteinerGraph() {
    vector<SteinerNode*> graph_nodes;
    map<ConverterNode*, SteinerNode*> node_map;

    for (ConverterNode* node : nodes_) {
      SteinerNode* graph_node = new SteinerNode(
          node->index(), node->x(), node->y(), node->isPin(), node->pin());
      graph_nodes.push_back(graph_node);
      node_map[node] = graph_node;
    }

    for (auto [node, graph_node] : node_map) {
      for (ConverterNode* neighbor : node->getRightNeighbors()) {
        graph_node->addNeighbor(node_map[neighbor]);
      }
      for (ConverterNode* neighbor : node->getLeftNeighbors()) {
        graph_node->addNeighbor(node_map[neighbor]);
      }
      for (ConverterNode* neighbor : node->getUpNeighbors()) {
        graph_node->addNeighbor(node_map[neighbor]);
      }
      for (ConverterNode* neighbor : node->getDownNeighbors()) {
        graph_node->addNeighbor(node_map[neighbor]);
      }
    }

    return new SteinerGraph(graph_nodes);
  }

  void splitDegree4Nodes(SteinerGraph* graph) {
    deque<SteinerNode*> nodes_to_split;
    for (SteinerNode* node : graph->nodes()) {
      nodes_to_split.push_back(node);
    }

    while (!nodes_to_split.empty()) {
      SteinerNode* node = nodes_to_split.front();
      nodes_to_split.pop_front();
      if (node->neighbors_.size() <= 3) {
        continue;
      }

      SteinerNode* new_node = new SteinerNode(/*index=*/graph->nodeCount(),
                                              /*x=*/node->x(),
                                              /*y=*/node->y(),
                                              /*is_pin=*/false);
      graph->addNode(new_node);

      vector<SteinerNode*> next_neighbors;
      for (int i = 0; node->countNeighbors(); i++) {
        SteinerNode* neighbor = node->getNeighbor(i);
        if (i <= 1) {
          next_neighbors.push_back(neighbor);
          continue;
        }

        if (neighbor->isNeighbor(node)) {
          neighbor->removeNeighbor(node);
        }
        neighbor->addNeighbor(new_node);
        new_node->addNeighbor(neighbor);
      }

      node->clearNeighbors();
      for (SteinerNode* neighbor : next_neighbors) {
        node->addNeighbor(neighbor);
      }
      node->addNeighbor(new_node);
      new_node->addNeighbor(node);
      nodes_to_split.push_back(new_node);
    }
  }

  SteinerGraph* convertToSteinerGraph() {
    initializeNodes();
    initializeEdges();
    steinerize();
    return makeSteinerGraph();
  }

  int runDFS(SteinerNode* node, map<int, int>& parent_map) {
    int length = 0;
    for (SteinerNode* neighbor : node->getNeighbors()) {
      if (parent_map.find(neighbor->index()) == parent_map.end()) {
        parent_map[neighbor->index()] = node->index();
        length +=
            abs(neighbor->x() - node->x()) + abs(neighbor->y() - node->y());
        length += runDFS(neighbor, parent_map);
      }
    }
    return length;
  }

  Tree convertToSteinerTree() {
    SteinerGraph* graph = convertToSteinerGraph();
    splitDegree4Nodes(graph);
    map<int, int> parent_map;
    int driver_index = restree_->driverIndex();
    SteinerNode* start_node = graph->getNode(driver_index);
    parent_map[start_node->index()] = start_node->index();
    int length = runDFS(start_node, parent_map);
    vector<Branch> branches;
    for (int i = 0; i < graph->nodeCount(); i++) {
      SteinerNode* node = graph->getNode(i);
      Branch branch;
      branch.x = node->x();
      branch.y = node->y();
      branch.n = parent_map[node->index()];
      branches.push_back(branch);
    }

    Tree tree;
    tree.deg = restree_->numPins();
    tree.length = length;
    tree.branch = branches;

    delete graph;
    return tree;
  }

  RESTree* restree_;
  vector<ConverterNode*> nodes_;
};

class RESTreeDetourEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getWeight(TDPin* pin) {
    double slack = pin->slack_;
    if (slack <= -10E-12) {
      return slack / -10E-12;
    } else if (slack <= 10E-12) {
      return 1.0;
    } else if (slack <= 30E-12) {
      return 1.0 - (slack - 10E-12) / 20E-12 * 0.8;
    } else {
      return 0.2;
    }
  }

  double getCost(RESTree* tree) {
    map<int, int> manhattan_distances =
        calculateManhattanDistancesFromDriver(tree);
    map<int, int> pathlengths = calculatePathlengthsFromDriver(tree);
    double total_detour_cost = 0;
    for (int i = 0; i < tree->numPins(); i++) {
      TDPin* pin = tree->getPin(i);
      double weight = getWeight(pin);
      int detour = pathlengths[i] - manhattan_distances[i];
      double detour_cost = weight * detour;
      total_detour_cost += detour_cost;
    }
    return total_detour_cost * weight();
  }

  map<int, int> calculateManhattanDistancesFromDriver(RESTree* tree) {
    map<int, int> distances;
    int drv_x = tree->x(tree->driverIndex());
    int drv_y = tree->y(tree->driverIndex());
    for (int i = 0; i < tree->numPins(); i++) {
      int x = tree->x(i);
      int y = tree->y(i);
      distances[i] = abs(x - drv_x) + abs(y - drv_y);
    }
    return distances;
  }

  map<int, int> calculatePathlengthsFromDriver(RESTree* tree) {
    SteinerGraph* graph = TreeConverter(tree).convertToSteinerGraph();
    int driver_index = tree->driverIndex();
    SteinerNode* driver_node = graph->getNode(driver_index);

    map<int, int> pathlengths;
    calculatePathlengthsFromDriverHelper(driver_node, 0, pathlengths);
    for (int i = tree->numPins(); i < graph->nodeCount(); i++) {
      pathlengths.erase(i);
    }
    return pathlengths;
  }

  void calculatePathlengthsFromDriverHelper(SteinerNode* node, int length,
                                            map<int, int>& pathlengths) {
    pathlengths[node->index()] = length;

    for (SteinerNode* neighbor : node->getNeighbors()) {
      if (pathlengths.find(neighbor->index()) == pathlengths.end()) {
        int additional_length =
            abs(neighbor->x() - node->x()) + abs(neighbor->y() - node->y());
        int new_length = length + additional_length;
        calculatePathlengthsFromDriverHelper(neighbor, new_length, pathlengths);
      }
    }
  }
};

class RESTreeOptimizer {
 public:
  RESTreeOptimizer(vector<RESTreeAbstractEvaluator*> evaluators) {
    evaluators_ = evaluators;
  }

  double optimize(RESTree* tree) {
    double prev_cost = getCost(tree);
    int N = tree->numPins();
    RES best_res = tree->getRES();
    vector<Point> pts;
    for (int i = 0; i < N; i++) {
      pts.push_back(Point(tree->x(i), tree->y(i)));
    }

    vector<vector<int>> neighbors = getNearestNeighbors(pts);

    double best_cost = prev_cost;
    int best_delete = -1;
    RE best_add;

    while (true) {
      prev_cost = best_cost;
      for (int i = 0; i < best_res.count(); i++) {
        RES new_res = best_res;
        int dv = new_res.get(i).first;
        int dh = new_res.get(i).second;
        new_res.erase(i);

        UnionFind uf(N);

        for (int j = 0; j < new_res.count(); j++) {
          int nv = new_res.get(j).first;
          int nh = new_res.get(j).second;
          uf.unite(nv, nh);
        }

        for (int nv = 0; nv < N; nv++) {
          for (int nh = 0; nh < N; nh++) {
            if (nv == dv and nh == dh) {
              continue;
            }
            if (uf.connected(nv, nh)) {
              continue;
            }
            new_res.add(RE(nv, nh));
            tree->updateRES(new_res);
            double new_cost = getCost(tree);
            if (new_cost < best_cost) {
              best_cost = new_cost;
              best_res = new_res;
              best_delete = i;
              best_add = RE(nv, nh);
            }
            new_res.erase(new_res.count() - 1);
          }
        }
      }
      if (best_cost < prev_cost) {
        best_res.erase(best_delete);
        best_res.add(best_add);
      } else {
        break;
      }
    }
    tree->updateRES(best_res);
    return best_cost;
  }

  double getCost(RESTree* tree) {
    double cost = 0.0;
    for (RESTreeAbstractEvaluator* evaluator : evaluators_) {
      cost += evaluator->getCost(tree);
    }
    return cost;
  }

  vector<vector<int>> getNearestNeighbors(const vector<Point>& pts) {
    vector<int> data;
    data.clear();

    const size_t pt_count = pts.size();

    vector<vector<int>> neighbors(pt_count);
    data.reserve(pt_count * 5);
    data.resize(pt_count * 2, std::numeric_limits<int>::max());
    data.resize(pt_count * 4, std::numeric_limits<int>::min());
    data.resize(pt_count * 5);
    int* const ur = &data[0];  // NOLINT
    int* const lr = &data[pt_count];
    int* const ul = &data[pt_count * 2];
    int* const ll = &data[pt_count * 3];
    int* const sorted = &data[pt_count * 4];

    // sort in y-axis
    std::iota(sorted, sorted + pt_count, 0);
    std::stable_sort(sorted, sorted + pt_count, [&pts](int i, int j) {
      return std::make_pair(pts[i].getY(), pts[i].getX()) <
             std::make_pair(pts[j].getY(), pts[j].getX());
    });

    // Compute neighbors going from bottom to top in Y
    for (int idx = 0; idx < pt_count; ++idx) {
      const int pt_idx = sorted[idx];
      const int pt_x = pts[pt_idx].getX();
      // Update upper neighbors of all pts below pt (below.y <= pt.y)
      for (int i = 0; i < idx; ++i) {
        const int below_idx = sorted[i];
        const int below_x = pts[below_idx].getX();
        if (below_x <= pt_x && pt_x < ur[below_idx]) {  // pt in ur
          neighbors[below_idx].push_back(pt_idx);
          ur[below_idx] = pt_x;
        } else if (ul[below_idx] < pt_x && pt_x < below_x) {  // pt in ul
          neighbors[below_idx].push_back(pt_idx);
          ul[below_idx] = pt_x;
        }
      }

      // Set all lower neighbors for 'pt' (below.y <= pt.y)
      for (int i = idx - 1; i >= 0; --i) {
        const int below_idx = sorted[i];
        const int below_x = pts[below_idx].getX();
        if (pt_x <= below_x && below_x < lr[pt_idx]) {  // below in lr
          neighbors[pt_idx].push_back(below_idx);
          lr[pt_idx] = below_x;
        } else if (ll[pt_idx] < below_x && below_x < pt_x) {  // below in ll
          neighbors[pt_idx].push_back(below_idx);
          ll[pt_idx] = below_x;
        }
      }
    }

    return neighbors;
  }

  vector<RESTreeAbstractEvaluator*> evaluators_;
};


class TimingDrivenSteinerTreeBuilder {
 public:
  TimingDrivenSteinerTreeBuilder();
  ~TimingDrivenSteinerTreeBuilder();

  void init(odb::dbDatabase* db, Logger* logger);
  void setGrids(int x_grid, int y_grid);
  void setVEdge(int x, int y, int cap, int usage, int red);
  void setHEdge(int x, int y, int cap, int usage, int red);

  void optimize();
  void reserveNets(int n);
  void initializeRESTrees();
  void addOrUpdateNet(odb::dbNet* dbnet, const std::vector<int>& x,
                      const std::vector<int>& y, bool is_clock,
                      int driver_index, const std::vector<double>& slack,
                      const std::vector<double>& arrival_time);
  void clearNets();
  void printNets() const;
  void printEdges() const;
  void buildSteinerTrees();

  vector<RES> runREST(const vector<vector<TDPoint>>& input_data);
  RESTree* generateRandomRESTree(int n_pins, int x_grid, int y_grid,
                                 double slack_mean, double slack_std);
  Tree testRESTree();
  void testEvaluators();
  void testAll();

 private:
  Logger* logger_;
  odb::dbDatabase* db_;

  OverflowManager* overflow_manager_;
  RESTreeOptimizer* restree_optimizer_;
  std::map<odb::dbNet*, int> net_index_map_;
  vector<odb::dbNet*> nets_;
  vector<TDNet*> td_nets_;
  vector<RESTree*> restrees_;
  vector<Tree> steiner_trees_;
};

}  // namespace stt