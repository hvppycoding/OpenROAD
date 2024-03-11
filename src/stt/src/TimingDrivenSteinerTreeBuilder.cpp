#include "stt/TimingDrivenSteinerTreeBuilder.h"

#include <map>
#include <random>
#include <string>
#include <ctime>
#include <fstream>
#include <iostream>

#include "odb/db.h"

namespace stt {

void RES::initialize_from_1d(const vector<int>& res) {
  assert(res.size() % 2 == 0);
  for (int i = 0; i < res.size(); i += 2) {
    res_.push_back(RE(res[i], res[i + 1]));
  }
}

TimingDrivenSteinerTreeBuilder::TimingDrivenSteinerTreeBuilder()
    : db_(nullptr), logger_(nullptr), x_grid_(0), y_grid_(0) {}

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
    odb::dbNet* dbnet, const std::vector<int>& x, const std::vector<int>& y,
    int driver_index, const std::vector<double>& slack,
    const std::vector<double>& arrival_time) {
  TDNet* net;
  if (net_map_.find(dbnet) == net_map_.end()) {
    net = new TDNet();
  } else {
    net = net_map_[dbnet];
  }
  net->dbnet_ = dbnet;
  net->n_pins_ = x.size();
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

void TimingDrivenSteinerTreeBuilder::printNets() const {
  for (auto net : net_map_) {
    odb::dbNet* dbnet = net.first;
    TDNet* tdnet = net.second;
    bool debug = true;
    if (debug) {
      logger_->setDebugLevel(utl::STT, "addNet", 3);
      std::string msg;
      msg += "Net ";
      msg += dbnet->getConstName();
      msg += " ";
      for (int i = 0; i < tdnet->n_pins_; i++) {
        msg += "(" + std::to_string(tdnet->x_[i]) + ", " +
               std::to_string(tdnet->y_[i]) + ") ";
      }
      debugPrint(logger_, utl::STT, "addNet", 2, msg);
      logger_->setDebugLevel(utl::STT, "addNet", 0);
    }
  }
}

void TimingDrivenSteinerTreeBuilder::printEdges() const {
  for (int y = 0; y < y_grid_; y++) {
    for (int x = 0; x < x_grid_; x++) {
      logger_->report("VEdge ({}, {}) cap: {} usage: {} red: {}", x, y,
                      v_edges_[y][x].cap, v_edges_[y][x].usage,
                      v_edges_[y][x].red);
      logger_->report("HEdge ({}, {}) cap: {} usage: {} red: {}", x, y,
                      h_edges_[y][x].cap, h_edges_[y][x].usage,
                      h_edges_[y][x].red);
    }
  }
}

void TimingDrivenSteinerTreeBuilder::buildSteinerTrees() {
  printEdges();
  printNets();
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

static std::string getCurrentTimeString() {
  std::time_t now = std::time(nullptr);
  std::tm* ptm = std::localtime(&now);
  char buffer[32];
  // Format: YYYYMMDD-HHMMSS
  std::strftime(buffer, 32, "%Y%m%d-%H%M%S", ptm);
  return std::string(buffer);
}

static void writeRESTInput(const vector<vector<TDPoint> >& input_data,
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

vector<RES> TimingDrivenSteinerTreeBuilder::runREST(const vector<vector<TDPoint> >& input_data) {
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

RESTree* TimingDrivenSteinerTreeBuilder::generateRandomRESTree(int n_pins = 10, int x_grid = 10,
                               int y_grid = 10, double slack_mean = 0.0,
                               double slack_std = 1.0) {
  std::random_device rd;
  std::mt19937 gen(rd());

  vector<int> xs;
  vector<int> ys;
  vector<double> slacks;
  std::uniform_int_distribution<> dis_x(0, x_grid - 1);
  std::uniform_int_distribution<> dis_y(0, y_grid - 1);
  std::normal_distribution<> dis_slack(slack_mean, slack_std);

  for (int i = 0; i < n_pins; i++) {
    xs.push_back(dis_x(gen));
    ys.push_back(dis_y(gen));
    slacks.push_back(dis_slack(gen));
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
  net->x_ = xs;
  net->y_ = ys;
  net->driver_index_ = driver_index;
  net->slack_ = slacks;
  net->arrival_time_.resize(n_pins, 0.0);

  vector<vector<TDPoint> > input_data;
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
  RESTree* res_tree = new RESTree(net, res[0], pins);
  return res_tree;
}

void TimingDrivenSteinerTreeBuilder::testRESTree() {
  logger_->report("Hello");
  RESTree* res_tree = generateRandomRESTree();
  string str = res_tree->toString();
  logger_->report("Generated RESTree: {}", str);
  delete res_tree;
}

void TimingDrivenSteinerTreeBuilder::testAll() {
  testRESTree();
}

}  // namespace stt
