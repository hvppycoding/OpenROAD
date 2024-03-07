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

class TDNet {
public:
  const odb::dbNet* dbnet_;
  std::vector<int> x_;
  std::vector<int> y_;
  int driver_index_;
  std::vector<double> slack_;
  std::vector<double> arrival_time_;
};

class TDEdge
{
public:
  TDEdge() : cap(0), usage(0), red(0) {}

  unsigned short cap;
  unsigned short usage;
  unsigned short red;
};

class TimingDrivenSteinerTreeBuilder 
{
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
  void buildSteinerTrees();

  Tree makeSteinerTree(odb::dbNet* net, const std::vector<int>& x,
                       const std::vector<int>& y, int drvr_index);

 private:
  Logger* logger_;
  odb::dbDatabase* db_;

  int x_grid_;
  int y_grid_;
  std::map<odb::dbNet*, TDNet*> net_map_;
  vector<vector<TDEdge> > h_edges_;
  vector<vector<TDEdge> > v_edges_;
};

}  // namespace tdr