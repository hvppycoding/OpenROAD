#pragma once

#include <map>
#include <vector>

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

using stt::Tree;
using utl::Logger;

class TDPoint {
 public:
  TDPoint(): x_(0), y_(0) {}
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
  int n_pins_;
  std::vector<int> x_;
  std::vector<int> y_;
  int driver_index_;
  std::vector<double> slack_;
  std::vector<double> arrival_time_;
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

  std::vector<RE>::iterator begin() { return res_.begin(); }
  std::vector<RE>::iterator end() { return res_.end(); }
  std::vector<RE>::const_iterator begin() const { return res_.begin(); }
  std::vector<RE>::const_iterator end() const { return res_.end(); }

 private:
  vector<RE> res_;
};

class RESTree {
 public:
  RESTree(TDNet* net, const RES& res, const vector<TDPin*>& pins) {
    net_ = net;
    res_ = res;
    pins_ = pins;
    n_pins_ = pins.size();
    driver_index_ = -1;
    for (int i = 0; i < pins.size(); i++) {
      if (pins[i]->is_driver_) {
        assert(driver_index_ == -1);
        driver_index_ = i;
      }
    }
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
  string toString() const {
    string s = "RESTree\n";
    for (int i = 0; i < res_.count(); i++) {
      RE re = res_.get(i);
      s += "Edge " + std::to_string(re.first) + " -> " + std::to_string(re.second) + "\n";
    }
    for (int i = 0; i < n_pins_; i++) {
      s += "Pin " + std::to_string(i) + " (" + std::to_string(x_[i]) + ", " +
           std::to_string(y_[i]) + ")\n";
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
  OverflowManager(int x_grid, int y_grid) {
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

 private:
  int x_grid_;
  int y_grid_;
  vector<vector<int> > hcapacity_map_;
  vector<vector<int> > vcapacity_map_;
  vector<vector<int> > husage_map_;
  vector<vector<int> > vusage_map_;
};

class RESTreeAbstractEvaluator {
 public:
  virtual double getCost(RESTree& tree) = 0;
};

class RESTreeLengthEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getCost(RESTree& tree) { return tree.length(); }
};

class RESTreeOverflowEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeOverflowEvaluator(OverflowManager* overflow_manager) {
    overflow_manager_ = overflow_manager;
  }
  double getCost(RESTree& tree) {
    int overflow = 0;

    for (int i = 0; i < tree.numPins(); i++) {
      int y = tree.y(i);
      int x_low = tree.x_low(i);
      int x_high = tree.x_high(i);
      overflow += overflow_manager_->countHOverflow(y, x_low, x_high);
    }

    for (int i = 0; i < tree.numPins(); i++) {
      int x = tree.x(i);
      int y_low = tree.y_low(i);
      int y_high = tree.y_high(i);
      overflow += overflow_manager_->countVOverflow(x, y_low, y_high);
    }
    return overflow;
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

enum class EdgeType { H2V_RIGHT, H2V_LEFT, V2H_UP, V2H_DOWN };

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
  bool operator()(const SteinerCandidate& a, const SteinerCandidate& b) {
    return a.gain_ < b.gain_;
  }
};

class SteinerNode {
 public:
};

class SteinerGraph {
 public:
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
      node_v->addHorizontalNeighbor(node_h);
      node_h->addVerticalNeighbor(node_v);
    }
  }

  SteinerGraph* convertToSteinerGraph() { return nullptr; }

  RESTree* restree_;
  vector<ConverterNode*> nodes_;
};

class RESTreeDetourEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getCost(RESTree& tree) {
    int detour = 0;
    for (int i = 0; i < tree.numPins(); i++) {
      detour += tree.x_high(i) - tree.x_low(i) + tree.y_high(i) - tree.y_low(i);
    }
    return detour;
  }
};

class TimingDrivenSteinerTreeBuilder {
 public:
  TimingDrivenSteinerTreeBuilder();
  ~TimingDrivenSteinerTreeBuilder();

  void init(odb::dbDatabase* db, Logger* logger);
  void setGrids(int x_grid, int y_grid);
  void setVEdge(int x, int y, int cap, int usage, int red);
  void setHEdge(int x, int y, int cap, int usage, int red);

  void addOrUpdateNet(odb::dbNet* dbnet, const std::vector<int>& x,
                      const std::vector<int>& y, int driver_index,
                      const std::vector<double>& slack,
                      const std::vector<double>& arrival_time);
  void clearNets();
  void printNets() const;
  void printEdges() const;
  void buildSteinerTrees();

  Tree makeSteinerTree(odb::dbNet* net, const std::vector<int>& x,
                       const std::vector<int>& y, int drvr_index);

  vector<RES> runREST(const vector<vector<TDPoint> >& input_data);
  RESTree* generateRandomRESTree(int n_pins, int x_grid, int y_grid, double slack_mean, double slack_std);
  void testRESTree();
  void testAll();

 private:
  Logger* logger_;
  odb::dbDatabase* db_;

  int x_grid_;
  int y_grid_;
  std::map<odb::dbNet*, TDNet*> net_map_;
  vector<vector<TDEdge> > h_edges_;
  vector<vector<TDEdge> > v_edges_;
};



}  // namespace stt