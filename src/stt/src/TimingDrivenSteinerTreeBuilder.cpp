#include "stt/TimingDrivenSteinerTreeBuilder.h"

#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <string>

#include "odb/db.h"

namespace stt {

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

void RES::initialize_from_1d(const vector<int>& res) {
  assert(res.size() % 2 == 0);
  for (int i = 0; i < res.size(); i += 2) {
    res_.push_back(RE(res[i], res[i + 1]));
  }
}

TimingDrivenSteinerTreeBuilder::TimingDrivenSteinerTreeBuilder()
    : db_(nullptr), logger_(nullptr) {
  overflow_manager_ = new OverflowManager();
  RESTreeDetourEvaluator* detour = new RESTreeDetourEvaluator();
  detour->setWeight(0.3);
  RESTreeLengthEvaluator* length = new RESTreeLengthEvaluator();
  length->setWeight(0.7);
  RESTreeOverflowEvaluator* overflow = new RESTreeOverflowEvaluator(overflow_manager_);
  overflow->setWeight(0.2);
  // std::vector<RESTreeAbstractEvaluator*> evaluators; // = {detour, length, overflow};
  // restree_optimizer_ = new RESTreeOptimizer(evaluators);
}

TimingDrivenSteinerTreeBuilder::~TimingDrivenSteinerTreeBuilder() {
  clearNets();
}

void TimingDrivenSteinerTreeBuilder::init(odb::dbDatabase* db, Logger* logger) {
  db_ = db;
  logger_ = logger;
  logger_->report("Timing-driven Steiner Tree Builder initialized");
}

void TimingDrivenSteinerTreeBuilder::setGrids(int x_grid, int y_grid) {
  overflow_manager_->init(x_grid, y_grid);
}

void TimingDrivenSteinerTreeBuilder::setVEdge(int x, int y, int cap, int usage,
                                              int red) {
  overflow_manager_->setVCapacity(x, y, cap);
}

void TimingDrivenSteinerTreeBuilder::setHEdge(int x, int y, int cap, int usage,
                                              int red) {
  overflow_manager_->setHCapacity(x, y, cap);
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
  if (net_index_map_.find(dbnet) == net_index_map_.end()) {
    net_index = td_nets_.size();
    net = new TDNet();
  } else {
    net_index = net_index_map_[dbnet];
    net = td_nets_[net_index];
  }

  net->dbnet_ = dbnet;
  net->is_clock_ = is_clock;
  net->n_pins_ = x.size();
  net->x_ = x;
  net->y_ = y;
  net->driver_index_ = driver_index;
  net->slack_ = slack;
  net->arrival_time_ = arrival_time;

  if (nets_.size() <= net_index) {
    nets_.resize(net_index + 1, nullptr);
  }
  nets_[net_index] = dbnet;

  if (td_nets_.size() <= net_index) {
    td_nets_.resize(net_index + 1, nullptr);
  }
  td_nets_[net_index] = net;

  if (restrees_.size() <= net_index) {
    restrees_.resize(net_index + 1, nullptr);
  }
  if (steiner_trees_.size() <= net_index) {
    steiner_trees_.resize(net_index + 1);
  }
}

void TimingDrivenSteinerTreeBuilder::clearNets() {
  for (auto tdnet : td_nets_) {
    delete tdnet;
  }
  td_nets_.clear();
}

void TimingDrivenSteinerTreeBuilder::printNets() const {
  for (auto net : net_index_map_) {
    int index = net.second;
    odb::dbNet* dbnet = net.first;
    TDNet* tdnet = td_nets_[index];
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

void TimingDrivenSteinerTreeBuilder::initializeRESTrees() {
  vector<int> initialized_net_index;
  vector<vector<TDPoint> > input_data;
  for (int i = 0; i < td_nets_.size(); i++) {
    TDNet* net = td_nets_[i];
    if (restrees_[i] != nullptr) {
      logger_->report("Net {} already has a RESTree", i);
      continue;
    }
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
    overflow_manager_->addUsage(restree);
  }
}

void TimingDrivenSteinerTreeBuilder::optimize() {
  for (int i = 0; i < restrees_.size(); i++) {
    RESTree* restree = restrees_[i];
    if (restree == nullptr) {
      continue;
    }
    overflow_manager_->removeUsage(restree);
    restree_optimizer_->optimize(restree);
    overflow_manager_->addUsage(restree);
  }
}

void TimingDrivenSteinerTreeBuilder::buildSteinerTrees() {
  initializeRESTrees();
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

vector<RES> TimingDrivenSteinerTreeBuilder::runREST(
    const vector<vector<TDPoint> >& input_data) {
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
