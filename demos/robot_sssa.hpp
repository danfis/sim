#ifndef _ROBOT_SSSA_HPP_
#define _ROBOT_SSSA_HPP_

#include <sim/component.hpp>
#include <sim/robot/sssa.hpp>


class SSSAComp : public sim::Component {
    sim::Sim *_sim;

    sim::robot::SSSA *_robot;

  public:
    SSSAComp();

    void init(sim::Sim *sim);
    void finish();
};

#endif
