#include "FastRoute.h"
#include "DataType.h"
#include "FastRoute.h"
#include "utl/Logger.h"

namespace grt {

using utl::GRT;

std::vector<Tree> FastRouteCore::timeDrivenSteinerTree()
{
    logger_->report("Time-driven Steiner tree construction");
    return std::vector<Tree>();
}

}  // namespace grt