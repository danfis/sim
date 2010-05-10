#ifndef _SIM_BULLET_SIM_HPP_
#define _SIM_BULLET_SIM_HPP_

#include "sim/sim.hpp"

namespace sim {

namespace bullet {

class Sim : public sim::Sim {
  public:
    Sim(VisWorld *visworld);
    virtual ~Sim();
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_SIM_HPP_ */
