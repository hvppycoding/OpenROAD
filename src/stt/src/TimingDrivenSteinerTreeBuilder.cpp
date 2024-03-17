#include "stt/TimingDrivenSteinerTreeBuilder.h"

#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <string>

#include "odb/db.h"

namespace stt {

static Logger* __logger__ = nullptr;

static void reportSteinerBranches(const stt::Tree& tree, Logger* logger) {
  for (int i = 0; i < tree.branchCount(); i++) {
    int x1 = tree.branch[i].x;
    int y1 = tree.branch[i].y;
    int parent = tree.branch[i].n;
    int x2 = tree.branch[parent].x;
    int y2 = tree.branch[parent].y;
    int length = abs(x1 - x2) + abs(y1 - y2);
    logger->report("{} ({} {}) neighbor {} length {}", i, x1, y1, parent,
                   length);
  }
}

UnionFind::UnionFind(int size) : parent(size), rank(size, 0) {
  for (int i = 0; i < size; i++) {
    parent[i] = i;
  }
}

int UnionFind::find(int p) {
  while (p != parent[p]) {
    parent[p] = parent[parent[p]];  // Path compression
    p = parent[p];
  }
  return p;
}

void UnionFind::unite(int p, int q) {
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

bool UnionFind::connected(int p, int q) { return find(p) == find(q); }

RES::RES() {}
RES::RES(const vector<int>& res) { initialize_from_1d(res); }
RES::RES(const RES& res) { res_ = res.res_; }
void RES::initialize_from_1d(const vector<int>& res) {
  assert(res.size() % 2 == 0);
  for (int i = 0; i < res.size(); i += 2) {
    res_.push_back(RE(res[i], res[i + 1]));
  }
}
RE RES::get(int i) const { return res_[i]; }
int RES::count() const { return res_.size(); }
void RES::add(int i, int j) { res_.push_back(std::make_pair(i, j)); }
void RES::add(const RE& re) { res_.push_back(re); }
void RES::erase(int i) { res_.erase(res_.begin() + i, res_.begin() + i + 1); }
string RES::toString() const {
  string s = "";
  for (int i = 0; i < res_.size(); i++) {
    s += "(" + std::to_string(res_[i].first) + ", " +
         std::to_string(res_[i].second) + "), ";
  }
  return s;
}

std::vector<RE>::iterator RES::begin() { return res_.begin(); }
std::vector<RE>::iterator RES::end() { return res_.end(); }
std::vector<RE>::const_iterator RES::begin() const { return res_.begin(); }
std::vector<RE>::const_iterator RES::end() const { return res_.end(); }

RESTree::RESTree(TDNet* net, const RES& res) {
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

void RESTree::updateRES(const RES& res) {
  res_ = res;
  initialize();
}

string RESTree::netName() {
  if (net_ == nullptr) {
    return "Unknown";
  }
  return string(net_->dbnet_->getConstName());
}

void RESTree::initialize() {
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

int RESTree::numPins() const { return n_pins_; }
TDPin* RESTree::getPin(int i) const { return pins_[i]; }
int RESTree::length() const { return length_; }
int RESTree::x(int i) const { return x_[i]; }
int RESTree::y(int i) const { return y_[i]; }
int RESTree::x_low(int i) const { return x_low_[i]; }
int RESTree::x_high(int i) const { return x_high_[i]; }
int RESTree::y_low(int i) const { return y_low_[i]; }
int RESTree::y_high(int i) const { return y_high_[i]; }
int RESTree::driverIndex() const { return driver_index_; }
string RESTree::toString() const {
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
const RES& RESTree::getRES() const { return res_; }

OverflowManager::OverflowManager() {}

void OverflowManager::init(int x_grid, int y_grid) {
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

void OverflowManager::setVCapacity(int x, int y, int cap) {
  vcapacity_map_[y][x] = cap;
}
void OverflowManager::setHCapacity(int x, int y, int cap) {
  hcapacity_map_[y][x] = cap;
}
int OverflowManager::countHOverflow(int y, int xl, int xr) {
  int overflow = 0;
  for (int x = xl; x < xr; x++) {
    if (husage_map_[y][x] > hcapacity_map_[y][x]) {
      overflow += 1;
    }
  }
  return overflow;
}

int OverflowManager::countVOverflow(int x, int yl, int yr) {
  int overflow = 0;
  for (int y = yl; y < yr; y++) {
    if (vusage_map_[y][x] > vcapacity_map_[y][x]) {
      overflow += 1;
    }
  }
  return overflow;
}

void OverflowManager::changeVUsage(int x, int yl, int yh, int delta /*= 1*/) {
  for (int y = yl; y < yh; y++) {
    vusage_map_[y][x] += delta;
  }
}

void OverflowManager::changeHUsage(int y, int xl, int xr, int delta /*= 1*/) {
  for (int x = xl; x < xr; x++) {
    husage_map_[y][x] += delta;
  }
}

void OverflowManager::addTreeUsage(const RESTree* tree) {
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

void OverflowManager::removeTreeUsage(const RESTree* tree) {
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

void OverflowManager::reportOverflow(Logger* logger) {
  int h_overflow = 0;
  int v_overflow = 0;
  for (int y = 0; y < y_grid_; y++) {
    for (int x = 0; x < x_grid_; x++) {
      if (husage_map_[y][x] > hcapacity_map_[y][x]) {
        h_overflow += 1;
      }
      if (vusage_map_[y][x] > vcapacity_map_[y][x]) {
        v_overflow += 1;
      }
    }
  }
  logger->report("H overflow: {}", h_overflow);
  logger->report("V overflow: {}", v_overflow);

  logger->report("H capacity");
  for (int y = 0; y < y_grid_; y++) {
    string s = "";
    for (int x = 0; x < x_grid_; x++) {
      s += std::to_string(hcapacity_map_[y][x]) + " ";
    }
    logger->report(s);
  }
  logger->report("H usage");
  for (int y = 0; y < y_grid_; y++) {
    string s = "";
    for (int x = 0; x < x_grid_; x++) {
      s += std::to_string(husage_map_[y][x]) + " ";
    }
    logger->report(s);
  }
  logger->report("V capacity");
  for (int y = 0; y < y_grid_; y++) {
    string s = "";
    for (int x = 0; x < x_grid_; x++) {
      s += std::to_string(vcapacity_map_[y][x]) + " ";
    }
    logger->report(s);
  }
  logger->report("V usage");
  for (int y = 0; y < y_grid_; y++) {
    string s = "";
    for (int x = 0; x < x_grid_; x++) {
      s += std::to_string(vusage_map_[y][x]) + " ";
    }
    logger->report(s);
  }
}

OverflowManager* OverflowManager::createRandom(int nx, int ny) {
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

RESTreeAbstractEvaluator::RESTreeAbstractEvaluator() { weight_ = 1.0; }
double RESTreeAbstractEvaluator::weight() const { return weight_; }
void RESTreeAbstractEvaluator::setWeight(double weight) { weight_ = weight; }

double RESTreeLengthEvaluator::getCost(RESTree* tree) {
  return tree->length() * weight();
}

RESTreeOverflowEvaluator::RESTreeOverflowEvaluator(
    OverflowManager* overflow_manager) {
  overflow_manager_ = overflow_manager;
}

double RESTreeOverflowEvaluator::getCost(RESTree* tree) {
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

ConverterNode::ConverterNode(int index, int x, int y, bool is_pin,
                             TDPin* pin /*= nullptr*/) {
  index_ = index;
  x_ = x;
  y_ = y;
  is_pin_ = is_pin;
  pin_ = pin;
}

void ConverterNode::removeNeighbor(const ConverterNode* neighbor) {
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

TDPin* ConverterNode::getPin() const {
  if (is_pin_) {
    return pin_;
  } else {
    return nullptr;
  }
}

void ConverterNode::addHorizontalNeighbor(ConverterNode* neighbor) {
  if (neighbor->x_ <= x_) {
    addLeftNeighbor(neighbor);
  } else {
    addRightNeighbor(neighbor);
  }
}

void ConverterNode::addVerticalNeighbor(ConverterNode* neighbor) {
  if (neighbor->y_ <= y_) {
    addDownNeighbor(neighbor);
  } else {
    addUpNeighbor(neighbor);
  }
}

void ConverterNode::addLeftNeighbor(ConverterNode* neighbor) {
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

void ConverterNode::addRightNeighbor(ConverterNode* neighbor) {
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

void ConverterNode::addUpNeighbor(ConverterNode* neighbor) {
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

void ConverterNode::addDownNeighbor(ConverterNode* neighbor) {
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

const vector<ConverterNode*>& ConverterNode::getRightNeighbors() const {
  return right_neighbors_;
}

const vector<ConverterNode*>& ConverterNode::getLeftNeighbors() const {
  return left_neighbors_;
}

const vector<ConverterNode*>& ConverterNode::getUpNeighbors() const {
  return up_neighbors_;
}

const vector<ConverterNode*>& ConverterNode::getDownNeighbors() const {
  return down_neighbors_;
}

int ConverterNode::index() const { return index_; }
int ConverterNode::x() const { return x_; }
int ConverterNode::y() const { return y_; }
bool ConverterNode::isPin() const { return is_pin_; }
TDPin* ConverterNode::pin() const { return pin_; }

SteinerCandidate::SteinerCandidate(int gain, int x, int y, ConverterNode* node,
                                   ConverterNode* op_node1,
                                   ConverterNode* op_node2,
                                   EdgeType edge_type) {
  update(gain, x, y, node, op_node1, op_node2, edge_type);
}

void SteinerCandidate::update(int gain, int x, int y, ConverterNode* node,
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

SteinerNode::SteinerNode(int index, int x, int y, bool is_pin,
                         TDPin* pin /*= nullptr*/) {
  index_ = index;
  x_ = x;
  y_ = y;
  is_pin_ = is_pin;
  pin_ = pin;
}

int SteinerNode::index() const { return index_; }
int SteinerNode::x() const { return x_; }
int SteinerNode::y() const { return y_; }
void SteinerNode::addNeighbor(SteinerNode* neighbor) {
  neighbors_.push_back(neighbor);
}
void SteinerNode::removeNeighbor(SteinerNode* neighbor) {
  neighbors_.erase(std::remove(neighbors_.begin(), neighbors_.end(), neighbor),
                   neighbors_.end());
}
SteinerNode* SteinerNode::getNeighbor(int i) const { return neighbors_[i]; }
vector<SteinerNode*> SteinerNode::getNeighbors() const { return neighbors_; }
bool SteinerNode::isNeighbor(SteinerNode* neighbor) const {
  return std::find(neighbors_.begin(), neighbors_.end(), neighbor) !=
         neighbors_.end();
}
int SteinerNode::countNeighbors() const { return neighbors_.size(); }
void SteinerNode::clearNeighbors() { neighbors_.clear(); }

SteinerGraph::SteinerGraph(vector<SteinerNode*> nodes) : nodes_(nodes) {}
SteinerGraph::~SteinerGraph() {
  for (SteinerNode* node : nodes_) {
    delete node;
  }
}

vector<SteinerNode*> SteinerGraph::nodes() const { return nodes_; }
SteinerNode* SteinerGraph::getNode(int i) { return nodes_[i]; }
int SteinerGraph::nodeCount() const { return nodes_.size(); }
void SteinerGraph::addNode(SteinerNode* node) { nodes_.push_back(node); }

string SteinerGraph::toString() const {
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

TreeConverter::TreeConverter(RESTree* tree) : restree_(tree) {}

void TreeConverter::initializeNodes() {
  nodes_.clear();
  for (int i = 0; i < restree_->numPins(); i++) {
    TDPin* pin = restree_->getPin(i);
    ConverterNode* node = new ConverterNode(i, pin->x_, pin->y_, true, pin);
    nodes_.push_back(node);
  }
}

void TreeConverter::initializeEdges() {
  for (auto [nv, nh] : restree_->getRES()) {
    ConverterNode* node_v = nodes_[nv];
    ConverterNode* node_h = nodes_[nh];
    node_v->addVerticalNeighbor(node_h);
    node_h->addHorizontalNeighbor(node_v);
  }
}

SteinerCandidate TreeConverter::bestSteinerForNode(ConverterNode* node) {
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
      best.update(gain, node->x(), op2->y(), node, op1, op2, EdgeType::V2H_UP);
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

void TreeConverter::steinerize() {
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
      steiner_node = new ConverterNode(nodes_.size(), best.x_, best.y_, false);
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

SteinerGraph* TreeConverter::makeSteinerGraph() {
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

void TreeConverter::splitDegree4Nodes(SteinerGraph* graph) {
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
    for (int i = 0; i < node->countNeighbors(); i++) {
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

SteinerGraph* TreeConverter::convertToSteinerGraph() {
  initializeNodes();
  initializeEdges();
  steinerize();
  SteinerGraph* graph = makeSteinerGraph();
  return graph;
}

int TreeConverter::runDFS(SteinerNode* node, map<int, int>& parent_map) {
  int length = 0;
  for (SteinerNode* neighbor : node->getNeighbors()) {
    if (parent_map.find(neighbor->index()) == parent_map.end()) {
      parent_map[neighbor->index()] = node->index();
      length += abs(neighbor->x() - node->x()) + abs(neighbor->y() - node->y());
      length += runDFS(neighbor, parent_map);
    }
  }
  return length;
}

Tree TreeConverter::convertToSteinerTree() {
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

double RESTreeDetourEvaluator::getWeight(TDPin* pin) {
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

double RESTreeDetourEvaluator::getCost(RESTree* tree) {
  vector<int> manhattan_distances = calculateManhattanDistancesFromDriver(tree);
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

vector<int> RESTreeDetourEvaluator::calculateManhattanDistancesFromDriver(
    RESTree* tree) {
  vector<int> distances;
  int drv_x = tree->x(tree->driverIndex());
  int drv_y = tree->y(tree->driverIndex());
  for (int i = 0; i < tree->numPins(); i++) {
    int x = tree->x(i);
    int y = tree->y(i);
    distances.push_back(abs(x - drv_x) + abs(y - drv_y));
  }
  return distances;
}

map<int, int> RESTreeDetourEvaluator::calculatePathlengthsFromDriver(
    RESTree* tree) {
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

void RESTreeDetourEvaluator::calculatePathlengthsFromDriverHelper(
    SteinerNode* node, int length, map<int, int>& pathlengths) {
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

RESTreeOptimizer::RESTreeOptimizer(vector<RESTreeAbstractEvaluator*> evaluators,
                                   Logger* logger)
    : evaluators_(evaluators), logger_(logger) {}

double RESTreeOptimizer::optimize(RESTree* tree) {
  double prev_cost = getCost(tree);
  int N = tree->numPins();
  RES best_res = tree->getRES();

  logger_->report("Initial cost: " + std::to_string(prev_cost));
  logger_->report("Initial RES: " + best_res.toString());
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
        for (int nh : neighbors[nv]) {
          if (nv == dv and nh == dh) {
            continue;
          }
          if (uf.connected(nv, nh)) {
            continue;
          }
          new_res.add(RE(nv, nh));
          tree->updateRES(new_res);
          // logger_->report("new_res: " + new_res.toString());
          double new_cost = getCost(tree);
          if (new_cost < best_cost) {
            best_cost = new_cost;
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
  logger_->report("Best cost: " + std::to_string(best_cost));
  logger_->report("Best RES: " + best_res.toString());
  return best_cost;
}

double RESTreeOptimizer::getCost(RESTree* tree) {
  double cost = 0.0;
  for (RESTreeAbstractEvaluator* evaluator : evaluators_) {
    cost += evaluator->getCost(tree);
  }
  return cost;
}

vector<vector<int>> RESTreeOptimizer::getNearestNeighbors(
    const vector<Point>& pts) {
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

TimingDrivenSteinerTreeBuilder::TimingDrivenSteinerTreeBuilder()
    : db_(nullptr), logger_(nullptr), restree_optimizer_(nullptr), parasitics_src_(ParasiticsSrc::NONE)
    {}

TimingDrivenSteinerTreeBuilder::~TimingDrivenSteinerTreeBuilder() {
  clearNets();
}

void TimingDrivenSteinerTreeBuilder::init(odb::dbDatabase* db, Logger* logger) {
  db_ = db;
  logger_ = logger;
  __logger__ = logger;
  overflow_manager_ = new OverflowManager();
  detour_evaluator_ = new RESTreeDetourEvaluator();
  detour_evaluator_->setWeight(0.3);
  length_evaluator_ = new RESTreeLengthEvaluator();
  length_evaluator_->setWeight(0.7);
  overflow_evaluator_ = new RESTreeOverflowEvaluator(overflow_manager_);
  overflow_evaluator_->setWeight(0.2);
  std::vector<RESTreeAbstractEvaluator*> evaluators;
  evaluators.push_back(detour_evaluator_);
  evaluators.push_back(length_evaluator_);
  evaluators.push_back(overflow_evaluator_);
  restree_optimizer_ = new RESTreeOptimizer(evaluators, logger_);
  logger_->report("Timing-driven Steiner Tree Builder initialized");
}

void TimingDrivenSteinerTreeBuilder::setGrids(int x_grid, int y_grid) {
  overflow_manager_->init(x_grid, y_grid);
}

void TimingDrivenSteinerTreeBuilder::setParasiticsSrc(ParasiticsSrc parasitics_src) {
  parasitics_src_ = parasitics_src;
}

void TimingDrivenSteinerTreeBuilder::setVEdge(int x, int y, int cap, int usage,
                                              int red) {
  overflow_manager_->setVCapacity(x, y, cap - usage);
}

void TimingDrivenSteinerTreeBuilder::setHEdge(int x, int y, int cap, int usage,
                                              int red) {
  overflow_manager_->setHCapacity(x, y, cap - usage);
}

void TimingDrivenSteinerTreeBuilder::reserveNets(int n) {
  nets_.reserve(n);
  td_nets_.reserve(n);
  restrees_.reserve(n);
  steiner_trees_.reserve(n);
}

void TimingDrivenSteinerTreeBuilder::addOrUpdateNet(
    odb::dbNet* dbnet, const std::vector<int>& x, const std::vector<int>& y,
    bool is_clock, int driver_index, const std::vector<double>& slack,
    const std::vector<double>& arrival_time) {
  TDNet* net;
  int net_index;

  // Update net if it already exists
  if (net_index_map_.find(dbnet) != net_index_map_.end()) {
    net_index = net_index_map_[dbnet];
    net = td_nets_[net_index];
    net->driver_index_ = driver_index;
    net->slack_ = slack;
    net->arrival_time_ = arrival_time;

    for (int i = 0; i < net->n_pins_; i++) {
      TDPin* pin = net->pins_[i];
      pin->x_ = net->x_[i];
      pin->y_ = net->y_[i];
      pin->slack_ = net->slack_[i];
      pin->arrival_time_ = net->arrival_time_[i];
    }
    return;
  }
  net_index = td_nets_.size();
  net_index_map_[dbnet] = net_index;
  net = new TDNet();
  net->dbnet_ = dbnet;
  net->is_clock_ = is_clock;
  net->n_pins_ = x.size();
  net->x_ = x;
  net->y_ = y;
  net->driver_index_ = driver_index;
  net->slack_ = slack;
  net->arrival_time_ = arrival_time;

  vector<TDPin*> pins;
  for (int i = 0; i < net->n_pins_; i++) {
    TDPin* pin = new TDPin();
    pin->net_ = net;
    pin->index_ = i;
    pin->x_ = net->x_[i];
    pin->y_ = net->y_[i];
    if (i == driver_index) {
      pin->is_driver_ = true;
    } else {
      pin->is_driver_ = false;
    }
    pin->slack_ = net->slack_[i];
    pin->arrival_time_ = net->arrival_time_[i];
    pins.push_back(pin);
  }
  net->pins_ = pins;

  if (nets_.size() < net_index + 1) {
    nets_.resize(net_index + 1, nullptr);
  }
  nets_[net_index] = dbnet;

  if (td_nets_.size() < net_index + 1) {
    td_nets_.resize(net_index + 1, nullptr);
  }
  td_nets_[net_index] = net;

  if (restrees_.size() < net_index + 1) {
    restrees_.resize(net_index + 1, nullptr);
  }
  if (steiner_trees_.size() < net_index + 1) {
    steiner_trees_.resize(net_index + 1);
  }
}

Tree TimingDrivenSteinerTreeBuilder::getSteinerTree(odb::dbNet* dbnet) {
  int net_index = net_index_map_[dbnet];
  logger_->report("Getting Steiner Tree for net {}", net_index);
  reportSteinerTree(steiner_trees_[net_index], logger_);
  return steiner_trees_[net_index];
}

void TimingDrivenSteinerTreeBuilder::clearNets() {
  for (auto tdnet : td_nets_) {
    if (tdnet != nullptr) delete tdnet;
  }
  for (auto restree : restrees_) {
    if (restree != nullptr) delete restree;
  }
  nets_.clear();
  td_nets_.clear();
  restrees_.clear();
  steiner_trees_.clear();
}

void TimingDrivenSteinerTreeBuilder::initializeRESTrees() {
  vector<int> initialized_net_index;
  vector<vector<TDPoint>> input_data;
  logger_->report("Initializing RESTrees");
  reportOverflow();

  for (int i = 0; i < td_nets_.size(); i++) {
    if (restrees_[i] != nullptr) {
      logger_->report("Net {} already has a RESTree", i);
      continue;
    }
    TDNet* net = td_nets_[i];
    vector<TDPoint> net_pin_pos;
    for (int j = 0; j < net->n_pins_; j++) {
      TDPoint point;
      point.x_ = net->x_[j];
      point.y_ = net->y_[j];
      net_pin_pos.push_back(point);
    }
    input_data.push_back(net_pin_pos);
    initialized_net_index.push_back(i);
  }

  vector<RES> reslist = runREST(input_data);

  for (int i = 0; i < initialized_net_index.size(); i++) {
    int net_idx = initialized_net_index[i];
    RES res = reslist[i];
    RESTree* restree = new RESTree(td_nets_[net_idx], res);
    restrees_[net_idx] = restree;
    overflow_manager_->addTreeUsage(restree);
  }

  logger_->report("RESTrees initialized");
  reportOverflow();
}

void TimingDrivenSteinerTreeBuilder::optimizeAll() {
  for (int i = 0; i < restrees_.size(); i++) {
    RESTree* restree = restrees_[i];
    if (restree == nullptr) {
      continue;
    }
    logger_->report("Optimizing RESTree for net {}", restree->netName());
    double prev_cost = reportRESTree(restree);
    overflow_manager_->removeTreeUsage(restree);
    restree_optimizer_->optimize(restree);
    overflow_manager_->addTreeUsage(restree);
    steiner_trees_[i] = TreeConverter(restree).convertToSteinerTree();
    logger_->report("Optimized RESTree for net {}", restree->netName());
    double new_cost = reportRESTree(restree);
    if (new_cost < prev_cost) {
      count_updated_trees_++;
    }
  }
}

void TimingDrivenSteinerTreeBuilder::buildSteinerTrees() {
  if (parasitics_src_ == ParasiticsSrc::PLACEMENT) {
    logger_->report("Building Initial Steiner Trees using placement parasitics");
    initializeRESTrees();
    for (int i = 0; i < restrees_.size(); i++) {
      RESTree* restree = restrees_[i];
      steiner_trees_[i] = TreeConverter(restree).convertToSteinerTree();
    }
  } else if (parasitics_src_ == ParasiticsSrc::GLOBAL_ROUTING) {
    logger_->report("Building Initial Steiner Trees using global routing parasitics");
    optimizeAll();
  }
}

int TimingDrivenSteinerTreeBuilder::getUpdatedTreesCount() {
  return count_updated_trees_;
}

void TimingDrivenSteinerTreeBuilder::resetUpdatedTreesCount() {
  count_updated_trees_ = 0;
}

static std::string getCurrentTimeString() {
  std::time_t now = std::time(nullptr);
  std::tm* ptm = std::localtime(&now);
  char buffer[32];
  // Format: YYYYMMDD-HHMMSS
  std::strftime(buffer, 32, "%Y%m%d-%H%M%S", ptm);
  return std::string(buffer);
}

static void writeRESTInput(const vector<vector<TDPoint>>& input_data,
                           const char* filepath) {
  std::ofstream out(filepath);
  assert(out);
  for (int i = 0; i < input_data.size(); i++) {
    for (int j = 0; j < input_data[i].size(); j++) {
      out << input_data[i][j].x_ << " " << input_data[i][j].y_ << " ";
    }
    out << "\n";
  }
}

static vector<RES> readRESTOutput(const char* filepath) {
  std::ifstream in(filepath);
  assert(in);
  string line;
  vector<RES> res;
  while (getline(in, line)) {
    vector<int> res_1d;
    std::istringstream iss(line);
    int nv, nh;
    while (iss >> nv >> nh) {
      res_1d.push_back(nv);
      res_1d.push_back(nh);
    }
    RES res_i;
    res_i.initialize_from_1d(res_1d);
    res.push_back(res_i);
  }
  return res;
}

vector<RES> TimingDrivenSteinerTreeBuilder::runREST(
    const vector<vector<TDPoint>>& input_data) {
  char cmd[1024];
  logger_->report("Running REST");
  const char* results_dir_ = getenv("RESULTS_DIR");
  std::string results_dir = results_dir_ ? results_dir_ : ".";
  string temp_dir = results_dir + "/temp";
  sprintf(cmd, "mkdir -p %s", temp_dir.c_str());
  logger_->report("Running command: {}", cmd);
  system(cmd);
  std::string current_time = getCurrentTimeString();
  string input_file = temp_dir + "/rest_input" + current_time + ".txt";
  writeRESTInput(input_data, input_file.c_str());
  string output_file = temp_dir + "/rest_output" + current_time + ".txt";
  string command_str = "runrest";
  command_str += " " + input_file;
  command_str += " --output " + output_file;
  system(command_str.c_str());
  vector<RES> res = readRESTOutput(output_file.c_str());
  assert(res.size() == input_data.size());
  return res;
}

void TimingDrivenSteinerTreeBuilder::reportOverflow() {
  overflow_manager_->reportOverflow(logger_);
}

double TimingDrivenSteinerTreeBuilder::reportRESTree(RESTree* tree) {
  double detour = detour_evaluator_->getCost(tree);
  double length = length_evaluator_->getCost(tree);
  double overflow = overflow_evaluator_->getCost(tree);
  double cost = detour + length + overflow;

  logger_->report("RESTree: {}", tree->netName());
  logger_->report("  Driver: {}", tree->driverIndex());
  logger_->report("  Pins: {}", tree->numPins());
  logger_->report("  RES: {}", tree->getRES().toString());
  logger_->report("  Detour: {}", detour);
  logger_->report("  Length: {}", length);
  logger_->report("  Overflow: {}", overflow);

  return cost;
}

RESTree* TimingDrivenSteinerTreeBuilder::generateRandomRESTree(
    int n_pins = 6, int x_grid = 10, int y_grid = 10, double slack_mean = 0.0,
    double slack_std = 0.0) {
  std::random_device rd;
  std::mt19937 gen(rd());

  vector<int> xs;
  vector<int> ys;
  vector<double> slacks;
  std::uniform_int_distribution<> dis_x(0, x_grid - 1);
  std::uniform_int_distribution<> dis_y(0, y_grid - 1);

  for (int i = 0; i < n_pins; i++) {
    xs.push_back(dis_x(gen));
    ys.push_back(dis_y(gen));
  }

  if (slack_std == 0) {
    for (int i = 0; i < n_pins; i++) {
      slacks.push_back(slack_mean);
    }
  } else {
    std::normal_distribution<> dis_slack(slack_mean, slack_std);
    for (int i = 0; i < n_pins; i++) {
      slacks.push_back(dis_slack(gen));
    }
  }

  vector<TDPin*> pins;
  for (int i = 0; i < n_pins; i++) {
    TDPin* pin = new TDPin();
    pin->x_ = xs[i];
    pin->y_ = ys[i];
    pin->slack_ = slacks[i];
    pin->arrival_time_ = 0.0;
    pin->is_driver_ = false;
    pins.push_back(pin);
  }

  std::uniform_int_distribution<> dis_driver(0, n_pins - 1);
  int driver_index = dis_driver(gen);
  pins[driver_index]->is_driver_ = true;

  TDNet* net = new TDNet();
  net->dbnet_ = nullptr;
  net->n_pins_ = n_pins;
  net->is_clock_ = false;
  net->x_ = xs;
  net->y_ = ys;
  net->driver_index_ = driver_index;
  net->slack_ = slacks;
  net->arrival_time_.resize(n_pins, 0.0);
  net->pins_ = pins;

  vector<vector<TDPoint>> input_data;
  for (int i = 0; i < n_pins; i++) {
    vector<TDPoint> pin_data;
    for (int j = 0; j < n_pins; j++) {
      TDPoint point;
      point.x_ = pins[j]->x_;
      point.y_ = pins[j]->y_;
      pin_data.push_back(point);
    }
    input_data.push_back(pin_data);
  }

  vector<RES> res = runREST(input_data);
  RESTree* res_tree = new RESTree(net, res[0]);
  return res_tree;
}

Tree TimingDrivenSteinerTreeBuilder::testRESTree() {
  logger_->report("Hello");
  RESTree* res_tree = generateRandomRESTree();
  string str = res_tree->toString();
  logger_->report("Generated RESTree: {}", str);

  TreeConverter* tree_converter = new TreeConverter(res_tree);
  SteinerGraph* graph = tree_converter->convertToSteinerGraph();
  logger_->report("Generated SteinerGraph: {}", graph->toString());
  delete graph;

  TreeConverter* tree_converter2 = new TreeConverter(res_tree);
  Tree tree = tree_converter2->convertToSteinerTree();
  reportSteinerBranches(tree, logger_);
  return tree;
}

void TimingDrivenSteinerTreeBuilder::testEvaluators() {
  OverflowManager* overflow_manager = OverflowManager::createRandom(20, 20);

  RESTreeDetourEvaluator* detour = new RESTreeDetourEvaluator();
  RESTreeLengthEvaluator* length = new RESTreeLengthEvaluator();
  RESTreeOverflowEvaluator* overflow =
      new RESTreeOverflowEvaluator(overflow_manager);

  RESTree* res_tree = generateRandomRESTree();
  string str = res_tree->toString();
  logger_->report("Generated RESTree: {}", str);

  logger_->report("Detour: {}", detour->getCost(res_tree));
  logger_->report("Length: {}", length->getCost(res_tree));
  logger_->report("Overflow: {}", overflow->getCost(res_tree));
}

void TimingDrivenSteinerTreeBuilder::testAll() { testRESTree(); }

}  // namespace stt
