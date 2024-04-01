#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include <deque>
#include <map>
#include <numeric>
#include <set>
#include <vector>

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

enum class ParasiticsSrc { NONE, PLACEMENT, GLOBAL_ROUTING };

class TDNet;

class UnionFind {
 private:
  std::vector<int> parent, rank;

 public:
  UnionFind(int size);
  int find(int p);
  void unite(int p, int q);
  bool connected(int p, int q);
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
  TDNet* net_ = nullptr;
  int index_ = -1;
  int x_ = 0;
  int y_ = 0;
  bool is_driver_ = false;
  double slack_ = 0.0;
  double arrival_time_ = 0.0;
  double cell_delay_ = 0.0;
  double load_pin_cap_ = 0.0;
  double load_wire_cap_ = 0.0;
  double pin_cap_ = 0.0;
};

class TDNet {
 public:
  odb::dbNet* dbnet_ = nullptr;
  bool is_clock_ = false;
  int n_pins_ = 0;
  std::vector<int> x_;
  std::vector<int> y_;
  int driver_index_ = -1;
  float slack_ = 0.0;
  std::vector<double> pins_slacks_;
  std::vector<double> arrival_time_;
  std::vector<TDPin*> pins_;
};

class TDEdge {
 public:
  unsigned short cap = 0;
  unsigned short usage = 0;
  unsigned short red = 0;
};

typedef std::pair<int, int> RE;

class RES {
 public:
  RES();
  RES(const vector<int>& res);
  RES(const RES& res);
  void initialize_from_1d(const vector<int>& res);
  RE get(int i) const;
  int count() const;
  void add(int i, int j);
  void add(const RE& re);
  void erase(int i);
  string toString() const;
  bool operator==(const RES& other) const;
  bool operator!=(const RES& other) const;

  std::vector<RE>::iterator begin();
  std::vector<RE>::iterator end();
  std::vector<RE>::const_iterator begin() const;
  std::vector<RE>::const_iterator end() const;

 private:
  vector<RE> res_;
};

class RESTree {
 public:
  RESTree(TDNet* net, const RES& res);
  void updateRES(const RES& res);
  string netName();
  void initialize();
  int numPins() const;
  TDPin* getPin(int i) const;
  int length() const;
  int x(int i) const;
  int y(int i) const;
  int x_low(int i) const;
  int x_high(int i) const;
  int y_low(int i) const;
  int y_high(int i) const;
  int driverIndex() const;
  string toString() const;
  const RES& getRES() const;

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
  OverflowManager();

  void init(int x_grid, int y_grid);
  void setVCapacity(int x, int y, int cap);
  void setHCapacity(int x, int y, int cap);
  int countHOverflow(int y, int xl, int xr);
  int countVOverflow(int x, int yl, int yr);
  void changeVUsage(int x, int yl, int yh, int delta = 1);
  void changeHUsage(int y, int xl, int xr, int delta = 1);
  void addTreeUsage(const RESTree* tree);
  void removeTreeUsage(const RESTree* tree);
  void reportOverflow(Logger* logger);
  static OverflowManager* createRandom(int nx, int ny);

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
  RESTreeAbstractEvaluator();
  virtual void initialize(RESTree* original) {};
  virtual double getCost(RESTree* tree) = 0;
  double weight() const;
  void setWeight(double weight);

 private:
  double weight_;
};

class RESTreeLengthEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getCost(RESTree* tree);
};

class RESTreeTotalLengthTimingEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeTotalLengthTimingEvaluator(double wire_r, double wire_c, int tile_size,
                                    int dbu_per_micron);
  void initialize(RESTree* original);
  double getCost(RESTree* tree);

 private:
  std::map<RESTree*, int> original_total_lengths_;
  double wire_r_;
  double wire_c_;
  int tile_size_;
  int dbu_per_micron_;
};

class RESTreeOverflowEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeOverflowEvaluator(OverflowManager* overflow_manager);
  double getCost(RESTree* tree);

 private:
  OverflowManager* overflow_manager_;
};

class ConverterNode {
 public:
  ConverterNode(int index, int x, int y, bool is_pin, TDPin* pin = nullptr);
  void removeNeighbor(const ConverterNode* neighbor);
  TDPin* getPin() const;
  void addHorizontalNeighbor(ConverterNode* neighbor);
  void addVerticalNeighbor(ConverterNode* neighbor);
  void addLeftNeighbor(ConverterNode* neighbor);
  void addRightNeighbor(ConverterNode* neighbor);
  void addUpNeighbor(ConverterNode* neighbor);
  void addDownNeighbor(ConverterNode* neighbor);
  const vector<ConverterNode*>& getRightNeighbors() const;
  const vector<ConverterNode*>& getLeftNeighbors() const;
  const vector<ConverterNode*>& getUpNeighbors() const;
  const vector<ConverterNode*>& getDownNeighbors() const;
  int index() const;
  int x() const;
  int y() const;
  bool isPin() const;
  TDPin* pin() const;

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
                   EdgeType edge_type);
  void update(int gain, int x, int y, ConverterNode* node,
              ConverterNode* op_node1, ConverterNode* op_node2,
              EdgeType edge_type);

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
  SteinerNode(int index, int x, int y, bool is_pin, TDPin* pin = nullptr);
  int index() const;
  int x() const;
  int y() const;
  void addNeighbor(SteinerNode* neighbor);
  void removeNeighbor(SteinerNode* neighbor);
  SteinerNode* getNeighbor(int i) const;
  vector<SteinerNode*> getNeighbors() const;
  bool isNeighbor(SteinerNode* neighbor) const;
  int countNeighbors() const;
  void clearNeighbors();

  int index_;
  int x_;
  int y_;
  bool is_pin_;
  TDPin* pin_;
  vector<SteinerNode*> neighbors_;
};

class SteinerGraph {
 public:
  SteinerGraph(vector<SteinerNode*> nodes);
  ~SteinerGraph();
  vector<SteinerNode*> nodes() const;
  SteinerNode* getNode(int i);
  int nodeCount() const;
  void addNode(SteinerNode* node);
  string toString() const;

 private:
  vector<SteinerNode*> nodes_;
};

class TreeConverter {
 public:
  TreeConverter(RESTree* tree);
  void initializeNodes();
  void initializeEdges();

  SteinerCandidate bestSteinerForNode(ConverterNode* node);
  void steinerize();
  SteinerGraph* makeSteinerGraph();
  void splitDegree4Nodes(SteinerGraph* graph);
  SteinerGraph* convertToSteinerGraph();
  int runDFS(SteinerNode* node, map<int, int>& parent_map);
  Tree convertToSteinerTree();

  RESTree* restree_;
  vector<ConverterNode*> nodes_;
};

vector<int> calculateManhattanDistancesFromDriver(RESTree* tree);
map<int, int> calculatePathlengthsFromDriver(RESTree* tree);
void calculatePathlengthsFromDriverHelper(SteinerNode* node, int length,
                                          map<int, int>& pathlengths);

class RESTreeDetourEvaluator : public RESTreeAbstractEvaluator {
 public:
  double getWeight(TDPin* pin);
  double getCost(RESTree* tree);
};

class RESTreeLengthTimingEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeLengthTimingEvaluator(double wire_r, double wire_c, int tile_size,
                               int dbu_per_micron);
  void initialize(RESTree* original);
  double slackToCost(double slack);
  double getCost(RESTree* tree);

 private:
  std::map<TDPin*, int> original_pathlengths_;
  double wire_r_;
  double wire_c_;
  int tile_size_;
  int dbu_per_micron_;
};

class RESTreeTimingDrivenEvaluator : public RESTreeAbstractEvaluator {
 public:
  RESTreeTimingDrivenEvaluator(double wire_r, double wire_c, int tile_size,
                               int dbu_per_micron);
  void initialize(RESTree* original);
  double slackToCostReduce(double prev_slack, double slack);
  double getCost(RESTree* tree);

 private:
  std::map<RESTree*, int> original_total_lengths_;
  std::map<TDPin*, int> original_pathlengths_;
  double wire_r_;
  double wire_c_;
  int tile_size_;
  int dbu_per_micron_;
};

class RESTreeOptimizer {
 public:
  RESTreeOptimizer(vector<RESTreeAbstractEvaluator*> evaluators,
                   Logger* logger);
  double optimize(RESTree* tree);
  void initializeEvaluators(RESTree* original);
  double getCost(RESTree* tree, bool debug = false);
  vector<vector<int>> getNearestNeighbors(const vector<Point>& pts);

  vector<RESTreeAbstractEvaluator*> evaluators_;
  Logger* logger_;
};

class TimingDrivenSteinerTreeBuilder {
 public:
  TimingDrivenSteinerTreeBuilder();
  ~TimingDrivenSteinerTreeBuilder();
  void init(odb::dbDatabase* db, Logger* logger);
  void setWireRC(double r, double c);
  void setGrids(int x_grid, int y_grid);
  void setTileSize(int tile_size);
  void setDbuPerMicron(int dbu_per_micron);
  void setParasiticsSrc(ParasiticsSrc parasitics_src);
  void saveRES();
  void useSavedRES(bool use_saved_res);
  void setVEdge(int x, int y, int cap, int usage, int red);
  void setHEdge(int x, int y, int cap, int usage, int red);
  void optimizeAll();
  void optimizeStep();
  void reserveNets(int n);
  void initializeRESTrees();
  void initializeOptimizer();
  void addOrUpdateNet(odb::dbNet* dbnet, const std::vector<int>& x,
                      const std::vector<int>& y, bool is_clock,
                      int driver_index, float slack,
                      const std::vector<double>& pin_slacks,
                      const std::vector<double>& arrival_time,
                      const std::vector<double>& cell_delays,
                      const std::vector<double>& load_pin_caps,
                      const std::vector<double>& load_wire_caps,
                      const std::vector<double>& pin_caps);
  Tree getSteinerTree(odb::dbNet* dbnet);
  void clearNets();
  void buildSteinerTrees();
  void buildSteinerTreesAll();
  int getUpdatedTreesCount();
  void resetUpdatedTreesCount();

  vector<RES> runREST(const vector<vector<TDPoint>>& input_data);
  void reportOverflow();
  double evaluateRESTree(RESTree* tree);
  double reportRESTree(RESTree* tree);
  RESTree* generateRandomRESTree(int n_pins, int x_grid, int y_grid,
                                 double slack_mean, double slack_std);
  Tree testRESTree();
  void testEvaluators();
  void testAll();

 private:
  Logger* logger_;
  ParasiticsSrc parasitics_src_;
  odb::dbDatabase* db_;
  OverflowManager* overflow_manager_;
  RESTreeOptimizer* restree_optimizer_;
  bool use_saved_res_ = false;
  int tile_size_ = 0;
  int dbu_per_micron_ = 0;
  int x_grid_ = 0;
  int y_grid_ = 0;
  double wire_r_ = 0.0;
  double wire_c_ = 0.0;
  std::map<odb::dbNet*, int> net_index_map_;
  std::set<odb::dbNet*> optimized_nets_;
  std::map<odb::dbNet*, RES> net_optimized_res_map_;
  std::map<odb::dbNet*, RES> saved_res_map_;
  vector<odb::dbNet*> nets_;
  vector<TDNet*> td_nets_;
  vector<RESTree*> restrees_;
  vector<Tree> steiner_trees_;
  RESTreeDetourEvaluator* detour_evaluator_;
  RESTreeLengthEvaluator* length_evaluator_;
  RESTreeOverflowEvaluator* overflow_evaluator_;
  RESTreeTotalLengthTimingEvaluator* total_length_timing_evaluator_;
  RESTreeLengthTimingEvaluator* length_timing_evaluator_;
  RESTreeTimingDrivenEvaluator* timing_driven_evaluator_;
  int count_updated_trees_;
};

}  // namespace stt