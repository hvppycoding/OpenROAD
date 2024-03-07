#include "stt/TimingDrivenSteinerTreeBuilder.h"

#include <map>
#include <string>

#include "odb/db.h"

namespace stt {

TimingDrivenSteinerTreeBuilder::TimingDrivenSteinerTreeBuilder()
    : x_grid_(0), y_grid_(0), db_(nullptr), logger_(nullptr) {}

TimingDrivenSteinerTreeBuilder::~TimingDrivenSteinerTreeBuilder() {
  clearNets();
}

void TimingDrivenSteinerTreeBuilder::init(odb::dbDatabase* db, Logger* logger) {
  db_ = db;
  logger_ = logger;
  logger_->report("Timing-driven Steiner Tree Builder initialized");
}

void TimingDrivenSteinerTreeBuilder::setGrids(int x_grid, int y_grid) {
  if (x_grid_ == x_grid && y_grid_ == y_grid) {
    return;
  }
  x_grid_ = x_grid;
  y_grid_ = y_grid;
  v_edges_.resize(0);
  h_edges_.resize(0);
  v_edges_.resize(y_grid_);
  h_edges_.resize(y_grid_);
  for (int y = 0; y < y_grid; y++) {
    v_edges_[y].resize(x_grid_);
    h_edges_[y].resize(x_grid_);
  }
}

void TimingDrivenSteinerTreeBuilder::setVEdge(int x, int y, int cap, int usage,
                                              int red) {
  v_edges_[y][x].cap = cap;
  v_edges_[y][x].usage = usage;
  v_edges_[y][x].red = red;
}

void TimingDrivenSteinerTreeBuilder::setHEdge(int x, int y, int cap, int usage,
                                              int red) {
  h_edges_[y][x].cap = cap;
  h_edges_[y][x].usage = usage;
  h_edges_[y][x].red = red;
}

void TimingDrivenSteinerTreeBuilder::addOrUpdateNet(
    odb::dbNet* dbnet, const std::vector<int>& x,
    const std::vector<int>& y, int driver_index,
    const std::vector<double>& slack, const std::vector<double>& arrival_time) {
  TDNet* net;
  if (net_map_.find(dbnet) == net_map_.end()) {
    net = new TDNet();
  } else {
    net = net_map_[dbnet];
  }
  net->dbnet_ = dbnet;
  net->x_ = x;
  net->y_ = y;
  net->driver_index_ = driver_index;
  net->slack_ = slack;
  net->arrival_time_ = arrival_time;
  net_map_[dbnet] = net;
}

void TimingDrivenSteinerTreeBuilder::clearNets() {
  for (auto net : net_map_) {
    delete net.second;
  }
  net_map_.clear();
}

void TimingDrivenSteinerTreeBuilder::buildSteinerTrees() {
  for (auto net : net_map_) {
    logger_->report("Building Steiner tree for net {}",
                    net.first->getConstName());
  }
}

Tree TimingDrivenSteinerTreeBuilder::makeSteinerTree(odb::dbNet* net,
                                                     const std::vector<int>& y,
                                                     const std::vector<int>& x,
                                                     int drvr_index) {
  std::string coordinates = "";
  for (int i = 0; i < x.size(); i++) {
    coordinates +=
        "(" + std::to_string(x[i]) + ", " + std::to_string(y[i]) + ") ";
  }
  logger_->report("makeSteinerTree Net {}: {}", net->getConstName(),
                  coordinates);

  if (net_map_.find(net) == net_map_.end()) {
    logger_->report("Net {} NOT initialized for timing-driven routing",
                    net->getConstName());
  } else {
    logger_->report("Net {} initialized for timing-driven routing",
                    net->getConstName());
  }
  return Tree();
}

}  // namespace stt